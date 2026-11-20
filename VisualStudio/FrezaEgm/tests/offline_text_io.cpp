// Test harness only. Does NOT test Windows UTF conversion or Windows file I/O.
#include "text_io.h"
#include <filesystem>
#include <fstream>
#include <codecvt>
#include <locale>
namespace freza {
std::wstring Utf8ToWide(const std::string& s) { return std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(s); }
std::string WideToUtf8(const std::wstring& s) { return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(s); }
void StripUtf8Bom(std::string& s) { if(s.compare(0,3,"\xef\xbb\xbf")==0) s.erase(0,3); }
bool ReadFileBytes(const std::wstring& p,std::string& s,std::wstring& e) {
    std::ifstream f(std::filesystem::u8path(WideToUtf8(p)),std::ios::binary);
    if(!f) { e=L"read failed"; return false; }
    s.assign(std::istreambuf_iterator<char>(f),{}); StripUtf8Bom(s); return true;
}
bool WriteFileBytes(const std::wstring& p,const std::string& s,std::wstring& e) {
    std::ofstream f(std::filesystem::u8path(WideToUtf8(p)),std::ios::binary);
    f.write(s.data(),s.size()); if(!f) {e=L"write failed";return false;} return true;
}
}
