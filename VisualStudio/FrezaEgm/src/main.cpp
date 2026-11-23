// main.cpp — точка входа GUI-приложения (подсистема Windows, WinMain).
#include "app_window.h"

#include <commctrl.h>

int APIENTRY wWinMain(HINSTANCE instance, HINSTANCE, LPWSTR, int showCommand)
{
    // Без этого на мониторах с масштабированием 125-150% окно выглядит размытым.
    ::SetProcessDPIAware();

    // Нужно для класса TRACKBAR (регулятор скорости).
    INITCOMMONCONTROLSEX commonControls{};
    commonControls.dwSize = sizeof(commonControls);
    commonControls.dwICC = ICC_BAR_CLASSES;
    ::InitCommonControlsEx(&commonControls);

    freza::AppWindow window;
    if (!window.Create(instance, showCommand))
    {
        ::MessageBoxW(nullptr, L"Не удалось создать главное окно приложения.", L"FrezaEgm", MB_ICONERROR);
        return 1;
    }

    return window.Run();
}
