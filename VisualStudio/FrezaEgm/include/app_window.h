// app_window.h — главное окно приложения (Win32 API, без сторонних библиотек).
#pragma once

#include <windows.h>

namespace freza {

class AppWindow {
public:
    bool Create(HINSTANCE instance, int showCommand);
    int Run();

private:
    HINSTANCE m_instance = nullptr;
    HWND m_hwnd = nullptr;
};

} // namespace freza
