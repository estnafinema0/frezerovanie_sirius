// Test-only adapter for Microsoft's array overload. Production sources unchanged.
#pragma once
#include <cwchar>
template<size_t N, class... A> int swprintf_s(wchar_t (&b)[N], const wchar_t* fmt, A... args) {
    return std::swprintf(b,N,fmt,args...);
}
