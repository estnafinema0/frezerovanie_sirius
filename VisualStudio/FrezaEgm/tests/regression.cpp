#include "cam.h"
#include "csv_io.h"
#include "egm_client.h"
#include "egm3.pb.h"
#include "pb_decode.h"
#include "text_io.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <chrono>
using freza::Point3;
namespace fs=std::filesystem;
namespace cam=freza::cam;
namespace {
int checks=0, failures=0;
void Check(bool b,const char* label) { ++checks; if(!b) ++failures; std::cout<<(b?"PASS ":"FAIL ")<<label<<'\n'; }
bool Near(double a,double b) {return std::abs(a-b)<1e-8;}
bool Same(Point3 a,Point3 b) {return Near(a.x,b.x)&&Near(a.y,b.y)&&Near(a.z,b.z);}
bool Same(const std::vector<Point3>& a,const std::vector<Point3>& b) {return a.size()==b.size()&&std::equal(a.begin(),a.end(),b.begin(),[](auto p,auto q){return Same(p,q);});}
std::wstring Wide(const fs::path& p) {
#ifdef _WIN32
 return p.wstring();
#else
 return freza::Utf8ToWide(p.u8string());
#endif
}
std::string Read(fs::path p) {std::ifstream f(p,std::ios::binary);return {std::istreambuf_iterator<char>(f),{}};}
void Write(fs::path p,const std::string& s) {std::ofstream f(p,std::ios::binary);f.write(s.data(),s.size());}
void Legacy(const fs::path& data,const fs::path& tmp) {
 const std::vector<Point3> expected{{750,0,600},{850,0,600},{850,100,600},{750,100,600},{750,0,600}};
 for(const char* name:{"trajectory.gcode","trajectory.xyz"}) {auto r=freza::LoadTrajectory(Wide(data/name));Check(r.ok&&Same(r.points,expected),name);}
 for(const char* name:{"points.csv","points_selftest.csv"}) {std::vector<Point3> p;std::wstring e;auto snapshot=expected;if(std::string(name)=="points_selftest.csv")for(auto& v:snapshot)v.z=300;Check(freza::ReadPointsCsv(Wide(data/name),p,e)&&Same(p,snapshot),name);}
 std::wstring e;std::vector<Point3> csv;
 auto unicode=tmp/fs::u8path(u8"точки с пробелами.csv");
 Check(freza::WritePointsCsv(Wide(unicode),expected,e)&&freza::ReadPointsCsv(Wide(unicode),csv,e)&&Same(csv,expected),"CSV roundtrip Unicode/spaces");
 auto file=tmp/"modal.gcode";
 Write(file,"G20\nG90\nG1 X1 Y0 Z0 F100\nG91\nG1 X1\nG2 X1 Y1 I1\n");
 auto modal=freza::LoadTrajectory(Wide(file));
 Check(modal.ok&&Same(modal.points,{{25.4,0,0},{50.8,0,0},{76.2,25.4,0}})&&!modal.warnings.empty(),"G20/G91, arc endpoint, F ignored regression");
 std::vector<Point3> dense;std::vector<int> dwell;std::vector<unsigned char> flags;
 freza::InterpolatePath(expected,2,25,dense,dwell,flags);
 bool exact=dense.size()==201&&dwell.size()==201&&flags.size()==201;
 for(size_t i=0;exact&&i<201;++i) {
   size_t side=i==0?0:(i-1)/50; double t=i==0?0:double((i-1)%50+1)/50;
   auto a=expected[side],b=expected[side+1];
   exact=Same(dense[i],{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,600})&&dwell[i]==(i==0?200:80)&&flags[i]==(i%50==0?1:0);
 }
 Check(exact,"all 201 absolute interpolated points, dwell times and corner flags");
 freza::InterpolatePath({{0,0,0},{0,0,0},{1,0,0}},.3,50,dense,dwell,flags);
 Check(Same(dense,{{0,0,0},{.25,0,0},{.5,0,0},{.75,0,0},{1,0,0}})&&dwell==std::vector<int>({200,5,5,5,5})&&flags==std::vector<unsigned char>({1,0,0,0,1}),"nonintegral steps and duplicate vertices regression");
 freza::InterpolatePath({{0,0,0},{.2,0,0}},0,0,dense,dwell,flags);
 Check(dense.size()==3&&dwell==std::vector<int>({200,1000,1000}),"legacy minimum step/feed unchanged");
}
void Serialization(const fs::path& data) {
 const std::vector<Point3> points{{0,0,0},{10,0,0},{750,0,600},{-1.25,0,3.5}};
 for(size_t i=0;i<points.size();++i) {
   unsigned char b[256]{};auto n=freza::EncodeEgmSensor(points[i],i==0?0:42,i==0?0:1000,b,sizeof b);
   std::string golden=Read(data/"cam"/("sensor-"+std::to_string(i)+".bin"));
   Check(n==golden.size()&&std::memcmp(b,golden.data(),n)==0,"serializer exact baseline bytes");
   EgmSensor s=EgmSensor_init_zero;auto stream=pb_istream_from_buffer(b,n);
   Check(pb_decode(&stream,EgmSensor_fields,&s)&&s.planned.cartesian.pos.has_x&&s.planned.cartesian.pos.has_y&&s.planned.cartesian.pos.has_z&&s.planned.cartesian.orient.has_u0&&s.planned.cartesian.orient.has_u1&&s.planned.cartesian.orient.has_u2&&s.planned.cartesian.orient.has_u3&&s.planned.cartesian.orient.u0==1.0,"XYZ zeros and quaternion explicit presence");
 }
}
void Cam(const fs::path& data,const fs::path& tmp) {
 cam::Options o;o.units=cam::Units::Millimetres;o.translationMm={750,0,599.5};
 std::string e;cam::Mesh m;cam::Result a,b;
 auto binary=data/"cam/Primer1.STL",ascii=data/"cam/Primer1-ascii.stl";
 Check(cam::LoadStl(binary,o,m,e)&&m.triangles.size()==12&&Same(m.bounds.min,{0,0,0})&&Same(m.bounds.max,{20,20,.5}),"Primer1 binary STL bounds");
 cam::Mesh ma;Check(cam::LoadStl(ascii,o,ma,e)&&ma.triangles.size()==12&&Same(ma.bounds.max,m.bounds.max),"ASCII STL bounds");
 auto unicode=tmp/fs::u8path(u8"пластина 20 мм.stl");Write(unicode,Read(binary));
 Check(cam::LoadStl(unicode,o,m,e),"STL Unicode/spaces path");
 auto bad=tmp/"bad.stl";cam::Mesh sentinel=m;
 auto reject=[&](std::string bytes,const char* label) {Write(bad,bytes);Check(!cam::LoadStl(bad,o,m,e)&&m.triangles.size()==sentinel.triangles.size()&&Same(m.bounds.max,sentinel.bounds.max),label);};
 reject("","empty file"); reject("solid bad\nendsolid bad\n","empty ASCII mesh");
 auto bytes=Read(binary);reject(bytes.substr(0,bytes.size()-1),"truncated binary count mismatch");
 auto n=bytes;n[80]=13;reject(n,"wrong binary triangle count");
 n=bytes;n[96]=0;n[97]=0;n[98]='\xC0';n[99]=char(0x7f);reject(n,"binary NaN coordinate");
 n=bytes;n[96]=0;n[97]=0;n[98]='\x80';n[99]=char(0x7f);reject(n,"binary Inf coordinate");
 n=bytes;std::copy(n.begin()+96,n.begin()+108,n.begin()+108);reject(n,"degenerate triangle");
 n=Read(ascii);n.replace(n.find("vertex ")+7,4,"nan ");reject(n,"ASCII NaN");
 n=Read(ascii);n.replace(n.find("vertex ")+7,4,"inf ");reject(n,"ASCII Inf");
 reject(Read(ascii)+"junk","trailing ASCII garbage");
 reject("solid a\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\n","truncated ASCII facet");
 reject(std::string(16*1024*1024+1,'x'),"file size limit");
 n=bytes;n.resize(84+50*100001);unsigned count=100001;for(int i=0;i<4;++i)n[80+i]=char(count>>(8*i));reject(n,"triangle count limit");
 n=bytes;n.replace(0,5,"solid");Write(bad,n);Check(cam::LoadStl(bad,o,m,e),"binary header starting with solid");
 auto invalid=[&](cam::Options opt,const char* label){Check(!cam::LoadStl(binary,opt,m,e),label);};
 auto opt=o;opt.units=cam::Units::Unspecified;invalid(opt,"explicit units required");
 opt=o;opt.scale=0;invalid(opt,"zero scale rejected");
 opt=o;opt.scale=std::numeric_limits<double>::infinity();invalid(opt,"infinite scale rejected");
 opt=o;opt.diameterMm=-4;invalid(opt,"negative diameter rejected");
 opt=o;opt.diameterMm=std::numeric_limits<double>::quiet_NaN();invalid(opt,"NaN diameter rejected");
 opt=o;opt.cutterLengthMm=0;invalid(opt,"zero cutter length rejected");
 opt=o;opt.samplingMm=0;invalid(opt,"zero sampling rejected");
 opt=o;opt.toleranceMm=1;invalid(opt,"excessive tolerance rejected");
 opt=o;opt.translationMm.z=std::numeric_limits<double>::infinity();invalid(opt,"infinite translation rejected");
 opt=o;opt.units=cam::Units::Inches;Check(cam::LoadStl(binary,opt,m,e)&&Near(m.bounds.max.x,508)&&Near(m.bounds.max.z,12.7),"inch conversion explicit");
 opt=o;opt.scale=2;Check(cam::LoadStl(binary,opt,m,e)&&Near(m.bounds.max.x,40),"explicit scale");
 if(!cam::Available()) {Check(!cam::Calculate(binary,o,a,e)&&e.find("unavailable")!=std::string::npos,"CAM off diagnostic and no fake result");return;}
 bool ok=cam::Calculate(binary,o,a,e);if(!ok)std::cerr<<e<<'\n';Check(ok,"real OpenCAMLib calculation");if(!ok)return;
 Check(cam::Calculate(ascii,o,b,e)&&Same(a.partPoints,b.partPoints)&&Same(a.basePoints,b.basePoints),"ASCII/binary same CAM output");
 Check(Same(a.partPoints,{{0,0,.5},{20,0,.5},{20,20,.5},{0,20,.5},{0,0,.5}}),"exact five-point closed upper contour");
 Check(a.contactCount>100&&a.basePoints.size()==5,"real dense contacts simplified only after collinearity");
 Check(Same(a.basePoints,{{750,0,600},{770,0,600},{770,20,600},{750,20,600},{750,0,600}}),"absolute TCP transform; no second tool offset");
 double length=0;for(size_t i=1;i<a.partPoints.size();++i){auto p=a.partPoints[i-1],q=a.partPoints[i];length+=std::hypot(q.x-p.x,q.y-p.y);}Check(Near(length,80),"one perimeter length 80 mm");
 Check(cam::ValidatePath(a,a.basePoints,e),"whole sparse contour valid");
 std::vector<Point3> dense;std::vector<int> dwell;std::vector<unsigned char> flags;
 freza::InterpolatePath(a.basePoints,2,50,dense,dwell,flags);
 Check(dense.size()==41&&cam::ValidatePath(a,dense,e),"entire transformed/interpolated piecewise-linear contour valid");
 Check(std::count(flags.begin(),flags.end(),1)==5&&dwell[0]==200&&dwell[1]==40,"CAM density does not add EGM corners or change dwell");
 auto altered=dense;altered[5].z-=.01;Check(!cam::ValidatePath(a,altered,e),"interior interpolated penetration rejected");
 altered=a.basePoints;altered[1]={770,1,600};Check(!cam::ValidatePath(a,altered,e),"edge endpoints with corner-cutting chord rejected");
 altered=a.basePoints;altered.back()={751,0,600};Check(!cam::ValidatePath(a,altered,e),"open contour rejected");
 altered=a.basePoints;altered.insert(altered.end(),a.basePoints.begin()+1,a.basePoints.end());Check(!cam::ValidatePath(a,altered,e),"second pass rejected");
 altered=dense;altered[4].x=std::numeric_limits<double>::quiet_NaN();Check(!cam::ValidatePath(a,altered,e),"nonfinite interpolated point rejected");
 Check(cam::ValidateInterpolation(a,2,50,e)&&!cam::ValidateInterpolation(a,std::numeric_limits<double>::quiet_NaN(),50,e)&&!cam::ValidateInterpolation(a,2,0,e),"interpolation input guard");
 opt=o;opt.diameterMm=8;Check(cam::Calculate(binary,opt,b,e)&&Same(a.partPoints,b.partPoints),"diameter changes contact calculation, center stays on boundary");
 opt=o;opt.samplingMm=1;Check(cam::Calculate(binary,opt,b,e)&&Same(a.partPoints,b.partPoints)&&b.contactCount<a.contactCount,"CAM sampling independent of retained vertices");
  // FaceZigzag: вся верхняя грань, шаг проходов производен от диаметра фрезы.
  opt=o;opt.strategy=cam::Strategy::FaceZigzag;
  Check(cam::Calculate(binary,opt,b,e)&&b.passCount==9&&Near(b.stepoverMm,2.5),"face raster: 9 passes, stepover 0.7*diameter");
  Check(b.partPoints.size()==18&&Near(b.partPoints.front().x,0)&&Near(b.partPoints.front().y,0)&&Near(b.partPoints.back().y,20)&&Near(b.partPoints.back().x,20),"face raster is a full-width serpentine to both edges");
  Check(cam::ValidatePath(b,b.basePoints,e),"face raster sparse path valid");
  std::vector<Point3> fd;std::vector<int> fw;std::vector<unsigned char> ff;
  freza::InterpolatePath(b.basePoints,2,50,fd,fw,ff);
  Check(cam::ValidatePath(b,fd,e),"interpolated face raster stays on the raster");
  auto offpath=fd;offpath[3].y+=1;Check(!cam::ValidatePath(b,offpath,e),"face point off the raster rejected");
  opt=o;opt.strategy=cam::Strategy::FaceZigzag;opt.diameterMm=20;Check(cam::Calculate(binary,opt,b,e)&&b.passCount==3,"larger cutter needs fewer raster passes");
  opt=o;opt.strategy=cam::Strategy::FaceZigzag;opt.stepoverFactor=1.5;Check(!cam::Calculate(binary,opt,b,e),"stepover above one diameter rejected");
 auto unsupported=[&](std::string text,const char* label){Write(bad,text);auto previous=a.basePoints;Check(!cam::Calculate(bad,o,a,e)&&Same(previous,a.basePoints),label);};
 n=Read(ascii);auto start=n.find("  facet");auto end=n.find("  endfacet",start)+std::string("  endfacet\n").size();n.erase(start,end-start);unsupported(n,"missing face rejected; output transactional");
 n=Read(ascii);auto pos=n.find("vertex 20.0 20.0 0.5");n.replace(pos,std::string("vertex 20.0 20.0 0.5").size(),"vertex 19.0 20.0 0.5");unsupported(n,"unknown nonrectangular model rejected");
 n=Read(ascii);start=n.find("  facet");end=n.find("  endfacet",start)+std::string("  endfacet\n").size();auto tri=n.substr(start,end-start);auto nextEnd=n.find("  endfacet",end)+std::string("  endfacet\n").size();n.replace(end,nextEnd-end,tri);unsupported(n,"duplicate/overlapping face rejected");
 opt=o;opt.scale=1000;opt.samplingMm=.01;Check(!cam::Calculate(binary,opt,b,e)&&e.find("200000")!=std::string::npos,"CAM sample resource limit");
}
}
int main() {
 auto data=fs::u8path(FREZA_DATA_DIR);
 auto tmp=fs::temp_directory_path()/("freza-regression-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));fs::create_directories(tmp);
 try {Legacy(data,tmp);Serialization(data);Cam(data,tmp);} catch(const std::exception& e) {Check(false,e.what());}
 fs::remove_all(tmp);
 std::cout<<checks-failures<<'/'<<checks<<" checks passed; CAM="<<cam::Available()<<'\n';return failures?1:0;
}
