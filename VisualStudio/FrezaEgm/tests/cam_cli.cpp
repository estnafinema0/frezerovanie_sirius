// Offline CLI: explicit units, scale and translation; never creates an EgmClient.
#include "cam.h"
#include "csv_io.h"
#include "text_io.h"
#include <iostream>
#include <stdexcept>
int Run(const std::vector<std::wstring>& args) {
    if(args.size()!=10 && args.size()!=11) {
        std::cerr<<"Usage: FrezaEgm_cam_cli input.stl output.csv mm|inch scale diameter_mm sampling_mm tx_mm ty_mm tz_mm [perimeter|face]\n";
        return 2;
    }
    try {
        auto number=[&](size_t i) { size_t n; double v=std::stod(args[i],&n); if(n!=args[i].size()) throw std::runtime_error("Invalid number"); return v; };
        freza::cam::Options o;
        o.units=args[3]==L"mm"?freza::cam::Units::Millimetres:args[3]==L"inch"?freza::cam::Units::Inches:freza::cam::Units::Unspecified;
        o.scale=number(4); o.diameterMm=number(5); o.samplingMm=number(6); o.translationMm={number(7),number(8),number(9)};
        if(args.size()==11) {
            if(args[10]==L"face") o.strategy=freza::cam::Strategy::FaceZigzag;
            else if(args[10]!=L"perimeter") throw std::runtime_error("Strategy must be perimeter or face");
        }
        freza::cam::Result r; std::string error;
#ifdef _WIN32
        std::filesystem::path input(args[1]);
#else
        auto input=std::filesystem::u8path(freza::WideToUtf8(args[1]));
#endif
        if(!freza::cam::Calculate(input,o,r,error)) { std::cerr<<error<<'\n';return 1; }
        std::wstring ioError;
        if(!freza::WritePointsCsv(args[2],r.basePoints,ioError)) {std::cerr<<freza::WideToUtf8(ioError)<<'\n';return 1;}
        std::cout<<freza::cam::Describe(r)<<"\nOffline output only; placement is not measured calibration.\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
#ifdef _WIN32
int wmain(int argc,wchar_t** argv) { return Run({argv,argv+argc}); }
#else
int main(int argc,char** argv) { std::vector<std::wstring> a;for(int i=0;i<argc;++i)a.push_back(freza::Utf8ToWide(argv[i]));return Run(a); }
#endif
