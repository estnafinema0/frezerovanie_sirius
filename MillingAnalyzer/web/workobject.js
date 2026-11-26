// R maps work-object axes to STEP axes; its transpose maps STEP to wobjIngot.
let stockFrame = null;
function makeStockFrame(origin, onX, onXY) {
  const ex = sub(onX, origin), ey = sub(onXY, origin);
  if (norm(ex) < 1e-8 || norm(ey) < 1e-8) throw Error('Точки привязки должны различаться.');
  const x = unit(ex), perpendicular = cross(x, unit(ey));
  if (norm(perpendicular) < 1e-6) throw Error('Три точки привязки лежат на одной прямой.');
  const z = unit(perpendicular), axes = [x, cross(z,x), z];
  // Rows of R^T are the three unit axes. Stable quaternion conversion near 180 degrees.
  const m = axes, tr = m[0][0]+m[1][1]+m[2][2];
  let q;
  if(tr>0) {
    const s=2*Math.sqrt(1+tr);
    q=[s/4,(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s];
  } else {
    const diag=m.map((r,i)=>r[i]), i=diag.indexOf(Math.max(...diag)), j=(i+1)%3, k=(i+2)%3;
    const s=2*Math.sqrt(1+m[i][i]-m[j][j]-m[k][k]);
    q=[(m[k][j]-m[j][k])/s,0,0,0];
    q[i+1]=s/4; q[j+1]=(m[j][i]+m[i][j])/s; q[k+1]=(m[k][i]+m[i][k])/s;
  }
  return {origin, axes, q:unit(q)};
}
function frameVector(f,v){return f.axes.map(axis=>dot(axis,v));}
function framePoint(f,p){return frameVector(f,sub(p,f.origin));}
function frameQuaternion(f,[w,x,y,z]){
  const [a,b,c,d]=f.q;
  return unit([a*w-b*x-c*y-d*z,a*x+b*w+c*z-d*y,a*y-b*z+c*w+d*x,a*z+b*y-c*x+d*w]);
}
function desktopCorner(){
  if(scene.program) return stockOrigin();
  const bounds=['xmin','ymin','zmax'].map(id=>$(id).value.trim());
  if(bounds.every(v=>v!=='' && Number.isFinite(Number(v)))) return bounds.map(Number);
  return stockOrigin();
}
function syncStockFrame(reset=false){
  if(reset){$('frameMode').value='stock';for(const id of ['frameO','frameX','frameY'])$(id).value='';}
  const corner=desktopCorner(), def=[corner,add(corner,[10,0,0]),add(corner,[0,10,0])];
  const custom=$('frameMode').value==='three';
  $('framePoints').hidden=!custom;
  try {
    const points=custom?['frameO','frameX','frameY'].map(id=>{
      const text=$(id).value.trim(), p=text.split(/\s+/).map(Number);
      if(p.length!==3||!p.every(Number.isFinite))throw Error('Введите три координаты через пробел для каждой точки.');
      return p;
    }):def;
    stockFrame=makeStockFrame(...points);
    const fmt=v=>v.map(k=>Number(k.toFixed(4))).join(' ');
    $('frameReport').textContent='O в STEP, мм: '+fmt(stockFrame.origin)+'\n'+stockFrame.axes.map((v,i)=>'+'+'XYZ'[i]+': '+fmt(v)).join('\n');
    if(!custom&&!scene.program && ['xmin','ymin','zmax'].some(id=>!$(id).value.trim()))
      $('frameReport').textContent+='\nУгол взят по габаритам модели. Для другого бруска задайте три точки.';
  }catch(e){stockFrame=null;$('frameReport').textContent=e.message;}
  $('exportStatus').textContent='';
  $('saveStockPath').disabled=!stockFrame||!face?.trajectory?.poses.length;
}
function drawStockFrame(){
  if(!stockFrame)return;
  label(stockFrame.origin,'0 wobjIngot','#fff');
  stockFrame.axes.forEach((axis,i)=>{
    const end=add(stockFrame.origin,mul(axis,extent*.1)), color=['#ff7979','#80d873','#7bacff'][i];
    arrow(stockFrame.origin,end,color);label(end,'+'+'XYZ'[i],color);
  });
}
function workObjectCsv(){
  if(!stockFrame)throw Error('Задайте систему координат заготовки.');
  const poses=face.trajectory?.poses;
  if(!poses || poses.length<2 || face.connectionError || face.error)
    throw Error('Нужна рассчитанная непрерывная траектория выбранной программы или грани.');
  const program=face.id===0;
  const lines=[program?'stage,operation,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,time_s,motion,collision,sweep_collision':
    'face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,nx,ny,nz,time_s,collision,sweep_collision,motion'];
  poses.forEach((p,i)=>{
    const prefix=program?[scene.program.stage[i],scene.program.operation[i]+1,i]:[face.id,0,i];
    const suffix=program?[p.t,p.motion,+p.collision,+p.sweepCollision]:
      [...frameVector(stockFrame,p.n),p.t,+p.collision,+p.sweepCollision,p.motion||0];
    const row=[...prefix,...framePoint(stockFrame,p.p),...frameQuaternion(stockFrame,tcpOrientation(p.q)),...suffix];
    if(!row.every(Number.isFinite))throw Error('Некорректная числовая поза '+i);
    lines.push(row.join(','));
  });
  return lines.join('\r\n')+'\r\n';
}
function installWorkObjectPanel(){
  $('stockFramePanel').hidden=false;
  $('plunge').closest('label').hidden=true;
  $('csv').hidden=true;
  $('frameMode').onchange=()=>{
    if($('frameMode').value==='three'&&!$('frameO').value){
      const o=desktopCorner();
      [o,add(o,[10,0,0]),add(o,[0,10,0])].forEach((v,i)=>$( ['frameO','frameX','frameY'][i]).value=v.join(' '));
    }
    syncStockFrame();modelInfo();draw();
  };
  for(const id of ['frameO','frameX','frameY','xmin','ymin','zmax'])
    $(id).addEventListener('input',()=>{syncStockFrame();modelInfo();draw();});
  $('saveStockPath').onclick=()=>{
    try{if(busy)throw Error('Дождитесь окончания расчёта.');download('prepared.poses.csv',workObjectCsv(),'text/csv');}
    catch(e){$('exportStatus').textContent=e.message;}
  };
}
