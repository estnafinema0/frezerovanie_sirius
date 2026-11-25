"""STEP -> server-owned CAM poses -> nanopb -> localhost UDP. No ABB connection."""
import csv
import io
import json
import math
import pathlib
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / 'experiments' / 'robotstudio_pose_force'))
from test_loopback import decode, integer, message, real

exe, fixture = sys.argv[1:3]
export_dir = pathlib.Path(sys.argv[3]) if len(sys.argv) > 3 else None
if export_dir:
    export_dir.mkdir(parents=True, exist_ok=True)
def pose_bytes(p, q):
    return message(1, b''.join(real(i+1,v) for i,v in enumerate(p))) + message(2,b''.join(real(i+1,v) for i,v in enumerate(q)))
def feedback(seq, pose):
    return message(1, integer(1,seq)+integer(2,seq*4)+integer(3,2))+message(2,message(2,pose))
def qmul(a,b):
    w,x,y,z=a; v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]
def conjugate(q): return [q[0],-q[1],-q[2],-q[3]]
def rotate(q,p): return qmul(qmul(q,[0]+p),conjugate(q))[1:]

with tempfile.TemporaryDirectory() as tmp:
    step=pathlib.Path(tmp)/'Пластина с пробелами.STP'
    raw=pathlib.Path(tmp)/'fixture.step'
    subprocess.run([fixture,'--fixture',str(raw)],check=True,stdout=subprocess.DEVNULL)
    raw.rename(step)  # Fixture CLI has narrow argv on Windows; upload still uses a Unicode path.
    with tempfile.TemporaryFile(mode='w+') as errors:
        proc=subprocess.Popen([exe,'--serve','--no-open'],stdout=subprocess.PIPE,stderr=errors,text=True)
        try:
            line=proc.stdout.readline();assert line.startswith('Milling Analyzer local UI: '),line
            root=line.strip().split(' ')[-1].rstrip('/')
            def api(route,params=None,body=b'',method='POST',expected=200,origin=None):
                if params: route+='?'+urllib.parse.urlencode(params)
                headers={'Content-Type':'application/octet-stream'}
                if origin:headers['Origin']=origin
                req=urllib.request.Request(root+route,data=body if method=='POST' else None,method=method,headers=headers)
                try:
                    with urllib.request.urlopen(req,timeout=45) as r:
                        data=r.read(); assert r.status==expected
                        return json.loads(data) if 'json' in r.headers['Content-Type'] else data.decode()
                except urllib.error.HTTPError as e:
                    data=e.read();assert e.code==expected,(e.code,data)
                    return json.loads(data)
            def status():return api('/egm/status',method='GET')
            def wait_state(states):
                until=time.monotonic()+4
                while time.monotonic()<until:
                    s=status()
                    if s['state'] in states:return s
                    time.sleep(.015)
                raise AssertionError(s)
            settings=dict(bind='127.0.0.1',peer='127.0.0.1',virtual=1,anchor=1)
            def free_port():
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
                    s.bind(('127.0.0.1',0));return s.getsockname()[1]
            def start(job,port):
                api('/egm/start',dict(settings,job=job,port=port));s=wait_state({'waiting','error'});assert s['state']=='waiting',s
            def exchange(sock,seq,pose,port):
                sock.sendto(feedback(seq,pose),('127.0.0.1',port))
                raw,source=sock.recvfrom(4096);assert source==('127.0.0.1',port)
                packet=decode(raw);header=decode(packet[1]);assert header[3]==3
                encoded=decode(packet[2])[2];cart=decode(encoded)
                assert set(cart)=={1,2},'No Euler or external axis command'
                xyz=decode(cart[1]);q=decode(cart[2]);assert set(xyz)=={1,2,3} and set(q)=={1,2,3,4}
                xyz=[xyz[i] for i in (1,2,3)];q=[q[i] for i in (1,2,3,4)]
                assert abs(sum(v*v for v in q)-1)<1e-10
                return encoded,xyz,q,header
            assert status()['state']=='idle'
            page=api('/',method='GET');assert 'id="egmPrepare"' in page and 'id="egmAnchor"' in page
            api('/egm/start',dict(settings,job=1,port=6510),expected=400)
            api('/egm/prepare',dict(face=1,revision=1),origin='https://example.com',expected=403)
            mesh=api('/load',dict(format='step'),step.read_bytes())
            floor=next(f['face'] for f in mesh['operationCatalog'] if f['horizontal'] and abs(f['z']-5)<1e-6)
            api('/egm/prepare',dict(face=floor,revision=mesh['trajectoryRevision']),expected=400)
            scene=api('/calculate',dict(face=floor,feed=20,spacing=2,step=2,clearance=2,length=8))
            prepared=api('/egm/prepare',dict(face=floor,revision=scene['trajectoryRevision']))
            assert prepared['state']=='prepared' and not prepared['active'] and prepared['sent']==0
            text=api('/egm/download',dict(job=prepared['job']),method='GET')
            if export_dir:
                (export_dir/'flat.step').write_bytes(step.read_bytes())
                (export_dir/'flat.poses.csv').write_text(text, encoding='utf-8')
                (export_dir/'flat-parameters.json').write_text(json.dumps(dict(face=floor,diameter=2.5,feed=20,spacing=2,step=2,clearance=2,length=8,preflight=prepared),indent=2),encoding='utf-8')
            rows=list(csv.DictReader(io.StringIO(text)));assert len(rows)==prepared['poses']
            assert all(r['pass_id']=='0' and r['face_id']==str(floor) for r in rows)
            first_cam=next(f for f in scene['faces'] if f['id']==floor)['trajectory']['poses'][0]
            first_xyz=[float(rows[0][k]) for k in ('part_x_mm','part_y_mm','part_z_mm')]
            assert abs(first_xyz[2]-(first_cam['p'][2]-5))<1e-6
            tcp=[float(rows[0][k]) for k in ('qw','qx','qy','qz')]
            assert rotate(tcp,[0,0,1])[2]<-.99999
            job=prepared['job']
            api('/egm/start',dict(settings,job=job,port=6510,virtual=0),expected=400)
            api('/egm/start',dict(settings,job=job,port=6510,anchor=0),expected=400)
            api('/egm/start',dict(settings,job=job,port=6510,peer='host.local'),expected=400)
            api('/egm/start',dict(settings,job=job,port=80),expected=400)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as occupied:
                occupied.bind(('127.0.0.1',0))
                api('/egm/start',dict(settings,job=job,port=occupied.getsockname()[1]))
                assert 'bind' in wait_state({'error'})['message']
            port=free_port()
            api('/egm/start',dict(settings,job=job,port=port,peer='127.0.0.2'))
            wait_state({'waiting'})
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as unexpected:
                unexpected.settimeout(.03)
                unexpected.sendto(feedback(0,pose_bytes([750,0,600],[0,1,0,0])),('127.0.0.1',port))
                try:unexpected.recv(4096);raise AssertionError('answered wrong IP')
                except socket.timeout:pass
            assert status()['state']=='waiting'
            api('/egm/stop');wait_state({'stopped'})
            calibrated = dict(settings, virtual=0, anchor=0, calibrated=1,
                              job=job, tx=100, ty=200, tz=300, qw=1, qx=0, qy=0, qz=0)
            api('/egm/start', dict(calibrated, port=6510, qw=2), expected=400)
            for mismatch in (True, False):
                port = free_port()
                api('/egm/start', dict(calibrated, port=port))
                wait_state({'waiting'})
                first = rows[0]
                p0 = [calibrated['t'+a]+float(first['part_'+a+'_mm']) for a in 'xyz']
                q0 = [float(first[k]) for k in ('qw','qx','qy','qz')]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
                    peer.settimeout(.2)
                    if mismatch:
                        peer.sendto(feedback(0, pose_bytes([p0[0]+1,*p0[1:]], q0)), ('127.0.0.1',port))
                        assert 'calibrated start' in wait_state({'error'})['message']
                        assert status()['sent'] == 0
                        try: peer.recv(4096); raise AssertionError('sent with invalid start')
                        except socket.timeout: pass
                    else:
                        _, p, q, _ = exchange(peer, 0, pose_bytes(p0,q0), port)
                        assert max(abs(a-b) for a,b in zip(p,p0)) < 1e-8
                        assert max(abs(a-b) for a,b in zip(q,q0)) < 1e-8
                        assert status()['translation'] == [100,200,300]
                        api('/egm/stop')
                        exchange(peer, 1, pose_bytes(p0,q0), port)
                        wait_state({'stopped'})
            port=free_port();start(job,port)
            api('/load',dict(format='step'),step.read_bytes(),expected=400)
            api('/calculate',dict(face=floor),expected=400)
            api('/reset-trajectory',dict(revision=scene['modelRevision']),expected=400)
            api('/egm/prepare',dict(face=floor,revision=scene['trajectoryRevision']),expected=400)
            api('/egm/start',dict(settings,job=job,port=port),expected=400)
            # No outbound packet before a valid explicit-presence robot pose.
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stranger:
                peer.bind(('127.0.0.1',0));stranger.bind(('127.0.0.1',0));peer.settimeout(.5);stranger.settimeout(.015)
                peer.sendto(b'bad protobuf',('127.0.0.1',port));peer.settimeout(.015)
                try:peer.recv(4096);raise AssertionError('answered malformed packet')
                except socket.timeout:pass
                peer.settimeout(.5)
                p0=[750,0,600];q0=[0,1,0,0];pose=pose_bytes(p0,q0)
                pose,p,q,h=exchange(peer,0,pose,port);assert max(abs(a-b) for a,b in zip(p,p0))<1e-8
                assert abs(q[1])>.999999 and abs(q[0])<1e-9
                stranger.sendto(feedback(100,pose),('127.0.0.1',port))
                try:stranger.recv(4096);raise AssertionError('accepted a second source port')
                except socket.timeout:pass
                peer.sendto(feedback(0,pose),('127.0.0.1',port));peer.settimeout(.015)
                try:peer.recv(4096);raise AssertionError('accepted duplicate sequence')
                except socket.timeout:pass
                peer.settimeout(.5)
                begin=time.monotonic();seq=1;last_sent=h[1]
                while time.monotonic()-begin<prepared['duration']+.35:
                    pose,p,q,h=exchange(peer,seq,pose,port);seq+=1
                    assert h[1]>last_sent;last_sent=h[1];time.sleep(.004)
                first=rows[0];last=rows[-1]
                fq=[float(first[k]) for k in ('qw','qx','qy','qz')]
                lq=[float(last[k]) for k in ('qw','qx','qy','qz')]
                r=qmul(q0,conjugate(fq))
                delta=rotate(r,[float(last[k])-float(first[k]) for k in ('part_x_mm','part_y_mm','part_z_mm')])
                assert max(abs(p[i]-(p0[i]+delta[i])) for i in range(3))<1e-8,(p,delta)
                expected_q=qmul(r,lq)
                assert abs(abs(sum(a*b for a,b in zip(q,expected_q)))-1)<1e-9
            completed=wait_state({'completed','error'});assert completed['state']=='completed',completed
            assert completed['anchored'] and completed['sent']>100
            # Lost feedback before the endpoint cannot masquerade as success.
            port=free_port();start(job,port)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
                peer.bind(('127.0.0.1',0));peer.settimeout(.5)
                exchange(peer,0,pose_bytes([750,0,600],[0,1,0,0]),port)
            assert 'lost' in wait_state({'error'})['message']
            # Waiting cancellation opens no motion; active stop freezes the last measured pose.
            port=free_port();start(job,port);api('/egm/stop');assert wait_state({'stopped'})['sent']==0
            port=free_port();start(job,port)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
                peer.bind(('127.0.0.1',0));peer.settimeout(.5)
                measured=pose_bytes([750,0,600],[0,1,0,0]);exchange(peer,0,measured,port)
                api('/egm/stop')
                for seq in range(1,10):
                    pose,p,q,h=exchange(peer,seq,measured,port);assert p==[750,0,600];time.sleep(.004)
                assert status()['state']=='holding'
            assert wait_state({'stopped'})['state']=='stopped'
            # Changing the calculated result invalidates both preparation and CSV download.
            updated=api('/calculate',dict(face=floor,feed=20,spacing=2,step=2,clearance=2,length=8))
            assert updated['trajectoryRevision']>scene['trajectoryRevision']
            api('/egm/start',dict(settings,job=job,port=port),expected=400)
            api('/egm/download',dict(job=job),method='GET',expected=400)
            api('/egm/prepare',dict(face=floor,revision=scene['trajectoryRevision']),expected=400)
            # Roughing program preserves plunge=4 in the actual canonical handoff CSV.
            program=api('/program',dict(xmin=-1,ymin=-1,zmin=0,xmax=21,ymax=11,zmax=6,plunge=1,mode='rough',clearance=2,length=8))
            pre=api('/egm/prepare',dict(face=0,revision=program['trajectoryRevision']))
            data=api('/egm/download',dict(job=pre['job']),method='GET')
            assert data.startswith('stage,operation,') and ',4,0,0\n' in data
            first_program=program['program']['trajectory']['poses'][0]['p']
            sent=next(csv.DictReader(io.StringIO(data)))
            for k,v,corner in zip(('part_x_mm','part_y_mm','part_z_mm'),first_program,(-1,-1,6)):
                assert abs(float(sent[k])-(v-corner))<1e-5
            if export_dir:
                (export_dir/'rough-program.poses.csv').write_text(data,encoding='utf-8')
            # Curved Bezier model sends changing tool orientation through the same path.
            api('/demo')
            curved=api('/calculate',dict(face=1,feed=10,spacing=2,step=1,clearance=2,length=8))
            pre=api('/egm/prepare',dict(face=1,revision=curved['trajectoryRevision']))
            port=free_port();start(pre['job'],port)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
                peer.bind(('127.0.0.1',0));peer.settimeout(.5)
                pose=pose_bytes([750,0,600],[0,1,0,0]);begin=time.monotonic();seq=0;changed=False
                first_q=curved['faces'][0]['trajectory']['poses'][0]['q']
                turn_time=next(p['t'] for p in curved['faces'][0]['trajectory']['poses'] if abs(sum(a*b for a,b in zip(p['q'],first_q)))<.9999)
                while time.monotonic()-begin<turn_time+.3:
                    pose,p,q,h=exchange(peer,seq,pose,port);seq+=1;changed|=abs(q[1])<.99999;time.sleep(.004)
                assert changed,'Curved surface orientation was not transmitted'
                api('/egm/stop');exchange(peer,seq,pose,port)
            wait_state({'stopped'})
            concave=api('/demo',dict(kind='bowl'))
            assert any(p['collision'] or p['sweepCollision'] for p in concave['faces'][0]['trajectory']['poses'])
            refused=api('/egm/prepare',dict(face=1,revision=concave['trajectoryRevision']),body=text.encode(),expected=400)
            assert 'collision' in refused['error'].lower(), 'Client body cannot clear server-owned collision flags'
            pre=api('/egm/prepare',dict(face=1,revision=concave['trajectoryRevision'],virtualReview=1))
            saved=api('/egm/download',dict(job=pre['job']),method='GET')
            assert any(r['collision']=='1' or r['sweep_collision']=='1' for r in csv.DictReader(io.StringIO(saved)))
            api('/egm/start',dict(calibrated,job=pre['job'],port=6510),expected=400)
            api('/egm/start',dict(settings,job=pre['job'],peer='192.0.2.1',port=6510),expected=400)
            port=free_port();start(pre['job'],port)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
                peer.settimeout(.5)
                pose,_,_,_=exchange(peer,0,pose_bytes([750,0,600],[1,0,0,0]),port)
                api('/egm/stop');exchange(peer,1,pose,port)
            wait_state({'stopped'})
            start_module=api('/egm/rapid-start',dict(job=pre['job']),method='GET')
            import re
            row=next(csv.DictReader(io.StringIO(saved)))
            p0=[float(row[k]) for k in ('part_x_mm','part_y_mm','part_z_mm')]
            q0=[float(row[k]) for k in ('qw','qx','qy','qz')]
            vals=[float(v) for v in re.findall(r'-?\d+\.\d+',start_module)]
            assert len(vals)==7 and all(abs(a-b)<1e-7 for a,b in zip(vals,p0+q0))
            for different_start in (True,False):
                port=free_port()
                api('/egm/start',dict(settings,job=pre['job'],port=port,workObject=1,anchor=0))
                wait_state({'waiting'})
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as peer:
                    peer.settimeout(.5)
                    if different_start:
                        peer.sendto(feedback(1,pose_bytes([p0[0]+2,*p0[1:]],q0)),('127.0.0.1',port))
                        assert wait_state({'error'})['sent']==0
                    else:
                        pose,_,_,_=exchange(peer,0,pose_bytes(p0,q0),port)
                        assert status()['translation']==[0,0,0] and status()['rotation']==[1,0,0,0]
                        api('/egm/stop');exchange(peer,1,pose,port)
                        wait_state({'stopped'})
            print('STEP -> CAM -> canonical CSV -> protobuf -> UDP: passed (XYZ, rotating quaternion, plunge=4, lifecycle and rejection). RobotStudio not used.')
        finally:
            proc.terminate();proc.wait(timeout=10)
