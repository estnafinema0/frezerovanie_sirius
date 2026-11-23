#include "geometry.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <iostream>
#include <stdexcept>
using namespace milling;
void check(bool b,const char* why){if(!b)throw std::runtime_error(why);}
int main(){try{
 Options o;o.validate();int n=0;
 for(V normal:{V(0,0,1),V(1,0,0),V(.2,.3,1).normalized()}){Q q=frame(normal,V(0,0,1));check((q*V::UnitZ()-normal).norm()<1e-9,"normal alignment");check(std::abs(q.norm()-1)<1e-12,"unit quaternion");check(std::abs(q.toRotationMatrix().determinant()-1)<1e-12,"right handed");auto next=frame(normal,V::Zero(),&q);check(q.dot(next)>.999,"fallback continuity");++n;}
 auto d=demo();auto face=TopoDS::Face(TopExp_Explorer(d.shape,TopAbs_FACE).Current());auto p=evaluate(face,.3,.7,o);check(std::abs(p.p.z()-(8*.3*.7+4*.7*.3))<1e-9,"Bezier value");auto rev=TopoDS::Face(face.Reversed());check((p.n+evaluate(rev,.3,.7,o).n).norm()<1e-9,"reversed normal");auto passes=raster(face,o);check(passes.size()==11||passes.size()==10,"physical row spacing");check(!passes.empty(),"raster exists");
 for(auto& pass:passes)for(size_t i=1;i<pass.poses.size();++i){auto&a=pass.poses[i-1];auto&b=pass.poses[i];check((a.p-b.p).norm()<=o.step+1e-8,"3D step");check(a.q.dot(b.q)>=0,"quaternion hemisphere");check((b.q*V::UnitZ()-b.n).norm()<1e-9,"tool normal");check(b.t>=a.t,"pass time");++n;}
 BRepBuilderAPI_MakePolygon outer;outer.Add(gp_Pnt(0,0,0));outer.Add(gp_Pnt(20,0,0));outer.Add(gp_Pnt(20,20,0));outer.Add(gp_Pnt(0,20,0));outer.Close();BRepBuilderAPI_MakeFace ring(outer.Wire());BRepBuilderAPI_MakePolygon inner;inner.Add(gp_Pnt(8,8,0));inner.Add(gp_Pnt(8,12,0));inner.Add(gp_Pnt(12,12,0));inner.Add(gp_Pnt(12,8,0));inner.Close();ring.Add(inner.Wire());auto trimmed=raster(ring.Face(),o);for(auto&pass:trimmed)for(auto&v:pass.poses)check(!(v.p.x()>8+1e-6&&v.p.x()<12-1e-6&&v.p.y()>8+1e-6&&v.p.y()<12-1e-6),"hole excluded");
 Pose origin;origin.p=V::Zero();origin.q=Q::Identity();Triangle tangent{V(-2,-2,0),V(2,-2,0),V(0,2,0),1},hit{V(-2,-2,1),V(2,-2,1),V(0,2,1),1};check(!triangleCollision(tangent,origin,1,20),"tangency is not penetration");check(triangleCollision(hit,origin,1,20),"interior triangle intersects cylinder");
 auto scene=analyze(d,o);check(!scene.faces[0].passes.empty(),"scene paths");check(!json(scene).empty(),"scene JSON");auto concave=analyze(demo(true),o);bool detected=false;for(auto&pass:concave.faces[0].passes)for(auto&v:pass.poses)detected|=v.collision;check(detected,"concave flat cutter gouge detected");
 bool threw=false;o.step=0;try{o.validate();}catch(...){threw=true;}check(threw,"bad parameters rejected");std::cout<<"Geometry / orientation / trim / collision checks passed; pose checks="<<n<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
