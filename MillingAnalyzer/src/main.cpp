#include "geometry.h"
#include "viewer_resource.h"
#include <fstream>
#include <iostream>
#include <locale>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#endif
int run(std::vector<std::string> args){
 try{
  milling::Options options;bool demo=false,bowl=false,open=false;std::filesystem::path input,output="milling-preview";
#ifdef _WIN32
  if(args.size()==1){wchar_t path[32768]={};OPENFILENAMEW dialog{};dialog.lStructSize=sizeof(dialog);dialog.lpstrFilter=L"STEP model\0*.step;*.stp\0\0";dialog.lpstrFile=path;dialog.nMaxFile=32768;dialog.Flags=OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR;dialog.lpstrTitle=L"Milling Analyzer: select STEP (Cancel = Bezier demo)";if(GetOpenFileNameW(&dialog))input=path;else demo=true;open=true;}
#else
  if(args.size()==1)demo=true;
#endif
  auto number=[](const std::string& s){size_t pos=0;double v=std::stod(s,&pos);if(pos!=s.size()||!std::isfinite(v))throw std::runtime_error("Invalid number");return v;};
  for(size_t i=1;i<args.size();++i){auto a=args[i];auto value=[&](){if(++i>=args.size())throw std::runtime_error("Missing argument");return args[i];};
   if(a=="--demo")demo=true;else if(a=="--bowl"){demo=true;bowl=true;}else if(a=="--open")open=true;
   else if(a=="--output")output=std::filesystem::u8path(value());
   else if(a=="--spacing")options.spacing=number(value());else if(a=="--step")options.step=number(value());else if(a=="--chord")options.chord=number(value());else if(a=="--angle")options.angleDeg=number(value());else if(a=="--diameter")options.diameter=number(value());else if(a=="--length")options.length=number(value());else if(a=="--feed")options.feed=number(value());
   else if(a=="--face"){double n=number(value());if(n<1||n>1000||std::floor(n)!=n)throw std::runtime_error("Invalid face ID");options.face=int(n);}
   else if(a=="--normal-sign"){double n=number(value());if(n!=1&&n!=-1)throw std::runtime_error("Normal sign must be +1 or -1");options.normalSign=int(n);}
   else if(a=="--help"){std::cout<<"MillingAnalyzer [file.STEP | --demo | --bowl] --output NEW_DIRECTORY [--face ID] [--spacing mm] [--step mm] [--chord mm] [--angle deg] [--diameter mm] [--length mm] [--feed mm/s] [--normal-sign 1|-1] [--open]\nOffline only. Result: preview.html and scene.json; pose CSV export in viewer.\n";return 0;}
   else if(a.rfind("--",0)==0)throw std::runtime_error("Unknown option: "+a);else{if(!input.empty())throw std::runtime_error("Only one model supported");input=std::filesystem::u8path(a);}
  }
  if(demo&&!input.empty())throw std::runtime_error("Choose either STEP or demo");if(!demo&&input.empty())throw std::runtime_error("Select STEP or --demo");options.validate();
  if(std::filesystem::exists(output))throw std::runtime_error("Output directory exists; choose a new --output directory");
  std::cout<<"Reading geometry and calculating poses (offline)...\n";
  auto model=demo?milling::demo(bowl):milling::loadStep(input);auto scene=milling::analyze(model,options);auto data=milling::json(scene);
  std::string html=viewerTemplate;auto mark=html.find("@@SCENE@@");if(mark==std::string::npos)throw std::runtime_error("Viewer resource missing");html.replace(mark,9,data);
  std::filesystem::create_directories(output);
  auto write=[&](const char* name,const std::string& text){std::ofstream f(output/name,std::ios::binary);f<<text;f.flush();if(!f)throw std::runtime_error("Output write failed");};
  write("scene.json",data);write("preview.html",html);
  size_t passes=0,poses=0;for(auto& f:scene.faces){passes+=f.passes.size();for(auto& p:f.passes)poses+=p.poses.size();std::cout<<"Face "<<f.id<<" "<<f.type<<": "<<f.passes.size()<<" passes "<<f.error<<'\n';}
  std::cout<<"Faces="<<scene.faces.size()<<", triangles="<<scene.mesh.size()<<", passes="<<passes<<", poses="<<poses<<"\nOpen "<<std::filesystem::absolute(output/"preview.html").u8string()<<"\nNo EGM output. Collision checks are sampled, not a machining clearance certificate.\n";
#ifdef _WIN32
  if(open)ShellExecuteW(nullptr,L"open",std::filesystem::absolute(output/"preview.html").c_str(),nullptr,nullptr,SW_SHOWNORMAL);
#else
  (void)open;
#endif
  return 0;
 }catch(const Standard_Failure& e){std::cerr<<"OCCT: "<<e.GetMessageString()<<'\n';return 1;}catch(const std::exception& e){std::cerr<<"ERROR: "<<e.what()<<'\n';return 1;}
}
#ifdef _WIN32
int wmain(int argc,wchar_t** argv){std::vector<std::string>a;for(int i=0;i<argc;++i)a.push_back(std::filesystem::path(argv[i]).u8string());return run(a);}
#else
int main(int argc,char** argv){return run(std::vector<std::string>(argv,argv+argc));}
#endif
