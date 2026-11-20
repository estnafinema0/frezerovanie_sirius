#include "cam.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#ifdef FREZA_HAS_CAM
#include <opencamlib/cylcutter.hpp>
#include <opencamlib/pathdropcutter.hpp>
#include <opencamlib/stlsurf.hpp>
#include <opencamlib/line.hpp>
#endif

namespace freza::cam {
namespace {
constexpr size_t MaxBytes = 16 * 1024 * 1024, MaxTriangles = 100000, MaxSamples = 200000;
bool Finite(Point3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
double Distance(Point3 a, Point3 b) { return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z); }
void Require(bool ok, const char* msg) { if (!ok) throw std::runtime_error(msg); }
double Factor(const Options& o) {
    Require(o.units == Units::Millimetres || o.units == Units::Inches, "Select STL units explicitly (mm or inches).");
    Require(std::isfinite(o.scale) && o.scale >= 1e-6 && o.scale <= 1e6, "Scale must be finite and between 1e-6 and 1e6.");
    return o.scale * (o.units == Units::Inches ? 25.4 : 1.0);
}
void CheckOptions(const Options& o) {
    Factor(o);
    Require(std::isfinite(o.diameterMm) && o.diameterMm >= 0.01 && o.diameterMm <= 1000, "Cutter diameter must be 0.01..1000 mm.");
    Require(std::isfinite(o.cutterLengthMm) && o.cutterLengthMm >= 0.01 && o.cutterLengthMm <= 10000, "Cutter length must be 0.01..10000 mm.");
    Require(std::isfinite(o.samplingMm) && o.samplingMm >= 0.01 && o.samplingMm <= 100, "CAM sampling must be 0.01..100 mm.");
    Require(std::isfinite(o.toleranceMm) && o.toleranceMm >= 1e-9 && o.toleranceMm <= 1e-4, "Tolerance must be 1e-9..1e-4 mm.");
    Require(Finite(o.translationMm) && std::max({std::abs(o.translationMm.x), std::abs(o.translationMm.y), std::abs(o.translationMm.z)}) <= 1e6, "Invalid translation (finite mm, absolute value <= 1e6 required).");
}
uint32_t U32(const char* p) {
    auto b = reinterpret_cast<const unsigned char*>(p);
    return uint32_t(b[0]) | (uint32_t(b[1])<<8) | (uint32_t(b[2])<<16) | (uint32_t(b[3])<<24);
}
double F32(const char* p) {
    uint32_t bits = U32(p); float f; std::memcpy(&f, &bits, 4);
    Require(std::isfinite(f), "STL contains NaN/Inf."); return f;
}
void Token(std::istream& s, const char* expected) {
    std::string t; Require(bool(s >> t) && t == expected, "Malformed ASCII STL grammar.");
}
Point3 ReadPoint(std::istream& s) {
    Point3 p; Require(bool(s >> p.x >> p.y >> p.z) && Finite(p), "Invalid/NaN/Inf STL coordinate or normal."); return p;
}
Bounds GetBounds(const std::vector<Point3>& p) {
    Require(!p.empty(), "Empty path."); Bounds b{p.front(),p.front()};
    for (auto v:p) {
        Require(Finite(v), "Non-finite path point.");
        b.min.x=std::min(b.min.x,v.x); b.min.y=std::min(b.min.y,v.y); b.min.z=std::min(b.min.z,v.z);
        b.max.x=std::max(b.max.x,v.x); b.max.y=std::max(b.max.y,v.y); b.max.z=std::max(b.max.z,v.z);
    } return b;
}
// Deliberately conservative domain: six rectangular faces, two triangles per face.
// Proves coverage rather than assuming that a mesh's bounding box is its contour.
void ProveBox(const Mesh& m, double tol) {
    Require(m.triangles.size()==12, "Unsupported surface: require a closed axis-aligned box with 12 triangles (2 per face).");
    const auto a=m.bounds.min, b=m.bounds.max;
    Require(b.x-a.x>100*tol && b.y-a.y>100*tol && b.z-a.z>100*tol, "Empty or too thin box.");
    std::array<std::vector<std::set<int>>,6> faces;
    std::map<std::pair<int,int>,int> edges;
    std::set<std::set<int>> unique;
    for (const auto& tri:m.triangles) {
        std::set<int> ids;
        for (auto p:tri) {
            Require((p.x==a.x || p.x==b.x) && (p.y==a.y || p.y==b.y) && (p.z==a.z || p.z==b.z), "Unsupported surface: vertices must be the eight exact corners of a flat axis-aligned box.");
            ids.insert((p.x==b.x?1:0)|(p.y==b.y?2:0)|(p.z==b.z?4:0));
        }
        Require(ids.size()==3 && unique.insert(ids).second, "Degenerate or duplicate STL triangle.");
        int face=-1;
        for(int axis=0;axis<3;++axis) for(int side=0;side<2;++side) {
            bool all=true; for(int id:ids) all = all && ((id>>axis)&1)==side;
            if(all) face=2*axis+side;
        }
        Require(face>=0, "Unsupported diagonal/internal face."); faces[face].push_back(ids);
        std::vector<int> v(ids.begin(),ids.end());
        for(int j=0;j<3;++j) ++edges[std::minmax(v[j],v[(j+1)%3])];
    }
    for(const auto& face:faces) {
        Require(face.size()==2, "Missing or overlapping box face.");
        std::vector<int> shared;
        std::set_intersection(face[0].begin(),face[0].end(),face[1].begin(),face[1].end(),std::back_inserter(shared));
        Require(shared.size()==2, "Incomplete box face.");
        int d=shared[0]^shared[1];
        Require(d==3 || d==5 || d==6, "Overlapping triangles: shared edge is not a face diagonal.");
    }
    for(auto e:edges) Require(e.second==2, "Non-manifold/open STL.");
}
Point3 Shift(Point3 p, Point3 t) { return {p.x+t.x,p.y+t.y,p.z+t.z}; }
std::array<Point3,5> Corners(Bounds b) {
    return {{{b.min.x,b.min.y,b.max.z},{b.max.x,b.min.y,b.max.z},{b.max.x,b.max.y,b.max.z},{b.min.x,b.max.y,b.max.z},{b.min.x,b.min.y,b.max.z}}};
}
}

bool Available() {
#ifdef FREZA_HAS_CAM
    return true;
#else
    return false;
#endif
}

bool LoadStl(const std::filesystem::path& path, const Options& o, Mesh& out, std::string& error) {
    try {
        CheckOptions(o);
        std::ifstream file(path,std::ios::binary|std::ios::ate);
        Require(bool(file), "Cannot open STL file.");
        auto size=file.tellg(); Require(size>0 && size<=static_cast<std::streamoff>(MaxBytes), "STL must be nonempty and <=16 MiB.");
        std::string bytes(static_cast<size_t>(size),'\0'); file.seekg(0);
        Require(bool(file.read(bytes.data(),size)), "Incomplete STL read.");
        Mesh m;
        // Binary headers may start with 'solid'; exact byte count takes precedence.
        bool binary=bytes.size()>=84 && 84ull+50ull*U32(bytes.data()+80)==bytes.size();
        if(binary) {
            size_t n=U32(bytes.data()+80); Require(n>0 && n<=MaxTriangles, "STL triangle count must be 1..100000.");
            m.triangles.reserve(n);
            for(size_t i=0;i<n;++i) {
                const char* p=bytes.data()+84+50*i;
                for(int j=0;j<3;++j) (void)F32(p+4*j); // validate normals too
                std::array<Point3,3> tri;
                for(int j=0;j<3;++j) tri[j]={F32(p+12+12*j),F32(p+16+12*j),F32(p+20+12*j)};
                m.triangles.push_back(tri);
            }
        } else {
            std::istringstream s(bytes); s.imbue(std::locale::classic());
            Token(s,"solid"); std::string line; std::getline(s,line);
            bool ended=false; std::string token;
            while(s>>token) {
                if(token=="endsolid") { std::getline(s,line); Require(!(s>>token), "Trailing ASCII STL data."); ended=true; break; }
                Require(token=="facet", "Malformed STL (or binary size/count mismatch).");
                Token(s,"normal"); (void)ReadPoint(s); Token(s,"outer"); Token(s,"loop");
                std::array<Point3,3> tri;
                for(auto& p:tri) { Token(s,"vertex"); p=ReadPoint(s); }
                Token(s,"endloop"); Token(s,"endfacet");
                Require(m.triangles.size()<MaxTriangles, "STL exceeds 100000 triangles."); m.triangles.push_back(tri);
            }
            Require(ended && !m.triangles.empty(), "Empty or truncated ASCII STL.");
        }
        const double f=Factor(o); std::vector<Point3> vertices;
        for(auto& tri:m.triangles) {
            for(auto& p:tri) {
                p={p.x*f,p.y*f,p.z*f};
                Require(Finite(p) && std::max({std::abs(p.x),std::abs(p.y),std::abs(p.z)})<=1e6, "STL coordinates exceed finite +/-1e6 mm limit.");
                vertices.push_back(p);
            }
            Point3 u{tri[1].x-tri[0].x,tri[1].y-tri[0].y,tri[1].z-tri[0].z};
            Point3 v{tri[2].x-tri[0].x,tri[2].y-tri[0].y,tri[2].z-tri[0].z};
            Require(Distance({u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x},{})>1e-12, "Degenerate STL triangle.");
        }
        m.bounds=GetBounds(vertices); out=std::move(m); error.clear(); return true;
    } catch(const std::exception& e) { error=e.what(); return false; }
}

bool Calculate(const std::filesystem::path& path, const Options& o, Result& out, std::string& error) {
    try {
        Require(Available(), "CAM unavailable. Build with FREZA_ENABLE_CAM=ON and pinned OpenCAMLib; coordinate/CSV loading remains available.");
        Mesh m; if(!LoadStl(path,o,m,error)) return false;
        ProveBox(m,o.toleranceMm);
        auto corners=Corners(m.bounds);
        double perimeter=2*((m.bounds.max.x-m.bounds.min.x)+(m.bounds.max.y-m.bounds.min.y));
        Require(perimeter/o.samplingMm+12<=MaxSamples, "CAM sampling exceeds 200000 points; increase sampling distance.");
        Result r; r.options=o; r.modelMm=m.bounds; r.triangleCount=m.triangles.size();
#ifdef FREZA_HAS_CAM
        ocl::STLSurf surface;
        for(const auto& t:m.triangles) surface.addTriangle(ocl::Triangle(ocl::Point(t[0].x,t[0].y,t[0].z),ocl::Point(t[1].x,t[1].y,t[1].z),ocl::Point(t[2].x,t[2].y,t[2].z)));
        ocl::CylCutter cutter(o.diameterMm,o.cutterLengthMm);
        // One OCL operation per straight side keeps corners explicit. No offset:
        // the CENTER of the flat cutter end follows the top face boundary.
        for(size_t side=0;side<4;++side) {
            auto a=corners[side], b=corners[side+1];
            ocl::Path p; p.append(ocl::Line(ocl::Point(a.x,a.y,a.z),ocl::Point(b.x,b.y,b.z)));
            ocl::PathDropCutter dc; dc.setSTL(surface); dc.setCutter(&cutter);
            dc.setSampling(o.samplingMm); dc.setZ(m.bounds.min.z-1.0); dc.setPath(&p); dc.run();
            auto pts=dc.getCLPoints(); Require(pts.size()>=2 && pts.size()<=MaxSamples, "OpenCAMLib returned invalid sample count.");
            r.contactCount+=pts.size();
            for(auto& cp:pts) {
                auto cc=cp.getCC();
                Require(cc.type!=ocl::NONE && Finite({cp.x,cp.y,cp.z}) && Finite({cc.x,cc.y,cc.z}), "OpenCAMLib did not find finite cutter contact.");
                Require(std::abs(cp.z-m.bounds.max.z)<=o.toleranceMm && std::abs(cc.z-m.bounds.max.z)<=o.toleranceMm, "Cutter contact is not on the top plane.");
                // Collinearity and range proof BEFORE reducing the side to endpoints.
                const double len=Distance(a,b);
                const double t=((cp.x-a.x)*(b.x-a.x)+(cp.y-a.y)*(b.y-a.y))/(len*len);
                Point3 projection{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z};
                Require(t>=-1e-12 && t<=1+1e-12 && Distance({cp.x,cp.y,cp.z},projection)<=o.toleranceMm, "Non-collinear CAM side: simplification refused.");
            }
            Require(Distance({pts.front().x,pts.front().y,pts.front().z},a)<=o.toleranceMm && Distance({pts.back().x,pts.back().y,pts.back().z},b)<=o.toleranceMm, "CAM endpoints differ from contour.");
            if(side==0) r.partPoints.push_back({pts.front().x,pts.front().y,pts.front().z});
            r.partPoints.push_back({pts.back().x,pts.back().y,pts.back().z});
        }
#endif
        for(auto p:r.partPoints) r.basePoints.push_back(Shift(p,o.translationMm));
        r.baseBounds=GetBounds(r.basePoints);
        if(!ValidatePath(r,r.basePoints,error)) return false;
        out=std::move(r); error.clear(); return true;
    } catch(const std::exception& e) { error=e.what(); return false; }
}

bool ValidatePath(const Result& r, const std::vector<Point3>& path, std::string& error) {
    try {
        CheckOptions(r.options);
        Require(path.size()>=5 && path.size()<=MaxSamples, "Invalid contour point count.");
        const double tol=r.options.toleranceMm;
        const auto c=Corners(r.modelMm);
        std::vector<Point3> p; p.reserve(path.size());
        for(auto b:path) {
            Require(Finite(b), "Non-finite base coordinate.");
            p.push_back({b.x-r.options.translationMm.x,b.y-r.options.translationMm.y,b.z-r.options.translationMm.z});
        }
        Require(Distance(p.front(),c.front())<=tol && Distance(p.back(),c.back())<=tol, "Contour must start/end at the first top corner.");
        // Every entire segment must remain on ONE side in forward order. Thus a
        // chord cutting a corner is rejected even if both endpoints are on edges.
        size_t side=0; double total=0;
        for(size_t i=1;i<p.size();++i) {
            auto a=p[i-1], b=p[i]; Require(Finite(a) && Finite(b), "Invalid coordinate.");
            if(Distance(a,b)<=1e-12) continue;
            if(side<3 && Distance(a,c[side+1])<=tol) ++side;
            auto s=c[side], e=c[side+1]; double len=Distance(s,e);
            Require(len>100*tol, "Invalid model bounds.");
            double ta=((a.x-s.x)*(e.x-s.x)+(a.y-s.y)*(e.y-s.y))/(len*len);
            double tb=((b.x-s.x)*(e.x-s.x)+(b.y-s.y)*(e.y-s.y))/(len*len);
            auto on=[&](Point3 v,double t) { return t>=-tol/len && t<=1+tol/len && Distance(v,{s.x+t*(e.x-s.x),s.y+t*(e.y-s.y),s.z})<=tol; };
            Require(on(a,ta) && on(b,tb) && tb>=ta, "Segment leaves top perimeter, cuts a corner, reverses, or penetrates the surface.");
            total+=Distance(a,b);
        }
        double perimeter=2*((r.modelMm.max.x-r.modelMm.min.x)+(r.modelMm.max.y-r.modelMm.min.y));
        Require(side==3 && std::abs(total-perimeter)<=tol*path.size(), "Path must make exactly one closed perimeter.");
        // Proven box is entirely at/below max.z. All segment points have max.z
        // within tolerance. A flat cylinder extending upward cannot penetrate it.
        error.clear(); return true;
    } catch(const std::exception& e) { error=e.what(); return false; }
}
bool ValidateInterpolation(const Result& r, double step, double feed, std::string& error) {
    if(!std::isfinite(step) || !std::isfinite(feed) || step<=0 || feed<=0 || feed>1e6) { error="Interpolation step/feed must be finite, positive; feed <=1e6 mm/s."; return false; }
    double estimate=1;
    for(size_t i=1;i<r.basePoints.size();++i) {
        double len=Distance(r.basePoints[i-1],r.basePoints[i]);
        double n=std::ceil(len/std::max(0.1,step)); estimate+=n;
        if(n>0 && len/n/std::max(0.1,feed)*1000>std::numeric_limits<int>::max()-1.0) { error="Interpolation dwell overflow."; return false; }
    }
    if(estimate>MaxSamples) { error="Interpolation exceeds 200000 points."; return false; }
    return ValidatePath(r,r.basePoints,error);
}
std::string Describe(const Result& r) {
    std::ostringstream s; s.imbue(std::locale::classic()); s<<std::fixed<<std::setprecision(6);
    auto p=[&](Point3 v) { s<<'('<<v.x<<", "<<v.y<<", "<<v.z<<')'; };
    s<<"STL units: "<<(r.options.units==Units::Millimetres?"mm":"inch")<<"; scale: "<<r.options.scale<<"; mm/unit: "<<Factor(r.options)<<"\r\n";
    s<<"Model mm: "; p(r.modelMm.min); s<<" .. "; p(r.modelMm.max);
    s<<"\r\nSize mm: "; p({r.modelMm.max.x-r.modelMm.min.x,r.modelMm.max.y-r.modelMm.min.y,r.modelMm.max.z-r.modelMm.min.z});
    s<<"\r\nCylCutter diameter/length mm: "<<r.options.diameterMm<<" / "<<r.options.cutterLengthMm<<"; CAM sampling mm: "<<r.options.samplingMm;
    s<<"\r\nCenter of flat end ON boundary; one closed top contour; no stock removal.\r\nBase TCP mm = part mm + "; p(r.options.translationMm);
    s<<"; rotation = identity; NO extra TCP length.\r\nBase trajectory bounds: "; p(r.baseBounds.min); s<<" .. "; p(r.baseBounds.max);
    s<<"\r\nTriangles: "<<r.triangleCount<<"; OCL contacts: "<<r.contactCount<<"; retained points: "<<r.basePoints.size();
    return s.str();
}
} // namespace freza::cam
