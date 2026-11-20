#include "app_window.h"

#include "csv_io.h"
#include "egm_client.h"
#include "text_io.h"
#include "trajectory.h"

#include <commdlg.h>
#include <commctrl.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace freza {
namespace {

constexpr wchar_t kClassName[] = L"FrezaEgmMainWindow";
constexpr wchar_t kTitle[] = L"FrezaEgm — фрезеровка через ABB EGM (protobuf 3 / nanopb)";

constexpr wchar_t kFilterTrajectory[] =
    L"Траектория (*.gcode;*.nc;*.ncl;*.xyz;*.txt)\0*.gcode;*.nc;*.ncl;*.xyz;*.txt\0Все файлы (*.*)\0*.*\0\0";
constexpr wchar_t kFilterCsv[] =
    L"CSV с точками (*.csv)\0*.csv\0Все файлы (*.*)\0*.*\0\0";

enum ControlId : int {
    kIdOpenTrajectory = 1001,
    kIdSaveCsv = 1002,
    kIdLoadCsv = 1003,
    kIdFileInfo = 1004,
    kIdPointInfo = 1005,
    kIdPort = 1010,
    kIdDwell = 1011,
    kIdPeriod = 1012,
    kIdInterp = 1013,
    kIdStep = 1014,
    kIdFeed = 1015,
    kIdSpeed = 1016,
    kIdStart = 1020,
    kIdStop = 1021,
    kIdDeltaX = 1030,
    kIdDeltaY = 1031,
    kIdDeltaZ = 1032,
    kIdSendPoint = 1033,
    kIdHome = 1034,
    kIdStatus = 1040,
    kIdLog = 1041,
};

struct WindowState {
    HWND hwnd = nullptr;
    HFONT font = nullptr;
    HFONT fontBold = nullptr;
    HBRUSH whiteBrush = nullptr;

    HWND hFileInfo = nullptr;
    HWND hPointInfo = nullptr;
    HWND hPort = nullptr;
    HWND hPeriod = nullptr;
    HWND hInterp = nullptr;
    HWND hStep = nullptr;
    HWND hFeed = nullptr;
    HWND hSpeed = nullptr;
    HWND hStart = nullptr;
    HWND hStop = nullptr;
    HWND hDeltaX = nullptr;
    HWND hDeltaY = nullptr;
    HWND hDeltaZ = nullptr;
    HWND hStatus = nullptr;
    HWND hWork = nullptr;
    HWND hLog = nullptr;

    std::vector<Point3> points;
    std::wstring trajectoryPath;
    EgmClient egm;
};

std::wstring Format(const wchar_t* format, ...)
{
    wchar_t buffer[1024] = {};
    va_list args;
    va_start(args, format);
    vswprintf_s(buffer, format, args);
    va_end(args);
    return std::wstring(buffer);
}

HFONT CreateUiFont(int pointSize, bool bold)
{
    HDC dc = ::GetDC(nullptr);
    const int height = -MulDiv(pointSize, ::GetDeviceCaps(dc, LOGPIXELSY), 72);
    ::ReleaseDC(nullptr, dc);

    return ::CreateFontW(height, 0, 0, 0, bold ? FW_SEMIBOLD : FW_NORMAL,
                         FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                         OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                         CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
}

HWND MakeControl(HWND parent, const wchar_t* cls, const wchar_t* text, DWORD style,
                 int x, int y, int width, int height, int id, HFONT font)
{
    HWND control = ::CreateWindowExW(0, cls, text, WS_CHILD | WS_VISIBLE | style,
                                     x, y, width, height, parent,
                                     reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
                                     nullptr, nullptr);
    if (control != nullptr && font != nullptr)
    {
        ::SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
    }
    return control;
}

void CreateControls(WindowState& state)
{
    HWND parent = state.hwnd;

    MakeControl(parent, L"BUTTON", L"1. Траектория", BS_GROUPBOX, 12, 10, 500, 140, -1, state.fontBold);
    state.hFileInfo = MakeControl(parent, L"STATIC", L"файл не загружен", SS_ENDELLIPSIS,
                                  26, 34, 470, 20, kIdFileInfo, state.font);
    MakeControl(parent, L"BUTTON", L"Открыть файл…", BS_PUSHBUTTON, 24, 60, 145, 28, kIdOpenTrajectory, state.font);
    MakeControl(parent, L"BUTTON", L"Сохранить CSV…", BS_PUSHBUTTON, 177, 60, 145, 28, kIdSaveCsv, state.font);
    MakeControl(parent, L"BUTTON", L"Загрузить CSV…", BS_PUSHBUTTON, 330, 60, 145, 28, kIdLoadCsv, state.font);
    state.hPointInfo = MakeControl(parent, L"STATIC", L"точек: 0", SS_LEFTNOWORDWRAP,
                                   26, 98, 470, 20, kIdPointInfo, state.font);
    MakeControl(parent, L"STATIC", L"G-code (G0/G1, G90/G91) или «x y z» построчно.",
                SS_ENDELLIPSIS, 26, 118, 470, 20, -1, state.font);

    MakeControl(parent, L"BUTTON", L"2. EGM-соединение", BS_GROUPBOX, 12, 158, 500, 152, -1, state.fontBold);
    MakeControl(parent, L"STATIC", L"Порт:", SS_LEFT, 26, 184, 40, 20, -1, state.font);
    state.hPort = MakeControl(parent, L"EDIT", L"6510", WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER,
                              68, 180, 76, 24, kIdPort, state.font);
    MakeControl(parent, L"STATIC", L"период, мс:", SS_LEFT, 176, 184, 66, 20, -1, state.font);
    state.hPeriod = MakeControl(parent, L"EDIT", L"4", WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER,
                                246, 180, 50, 24, kIdPeriod, state.font);
    state.hStart = MakeControl(parent, L"BUTTON", L"Запустить траекторию", BS_PUSHBUTTON,
                               24, 212, 190, 30, kIdStart, state.font);
    state.hStop = MakeControl(parent, L"BUTTON", L"Стоп", BS_PUSHBUTTON,
                              222, 212, 90, 30, kIdStop, state.font);
    state.hInterp = MakeControl(parent, L"BUTTON", L"Интерполировать", BS_AUTOCHECKBOX,
                                24, 254, 150, 22, kIdInterp, state.font);
    ::SendMessageW(state.hInterp, BM_SETCHECK, BST_CHECKED, 0);
    MakeControl(parent, L"STATIC", L"шаг, мм:", SS_LEFT, 186, 256, 56, 20, -1, state.font);
    state.hStep = MakeControl(parent, L"EDIT", L"2", WS_BORDER | ES_AUTOHSCROLL,
                              244, 252, 50, 24, kIdStep, state.font);
    MakeControl(parent, L"STATIC", L"скорость, мм/с:", SS_LEFT, 306, 256, 84, 20, -1, state.font);
    state.hFeed = MakeControl(parent, L"EDIT", L"50", WS_BORDER | ES_AUTOHSCROLL,
                              392, 252, 56, 24, kIdFeed, state.font);
    state.hSpeed = MakeControl(parent, TRACKBAR_CLASSW, L"",
                               WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS | TBS_HORZ,
                               24, 282, 470, 24, kIdSpeed, state.font);
    ::SendMessageW(state.hSpeed, TBM_SETRANGE, TRUE, MAKELPARAM(5, 200));
    ::SendMessageW(state.hSpeed, TBM_SETPOS, TRUE, 50);
    ::SendMessageW(state.hSpeed, TBM_SETTICFREQ, 25, 0);

    MakeControl(parent, L"BUTTON", L"3. Ручные команды", BS_GROUPBOX, 12, 318, 500, 112, -1, state.fontBold);
    MakeControl(parent, L"STATIC", L"dX:", SS_LEFT, 26, 344, 26, 20, -1, state.font);
    state.hDeltaX = MakeControl(parent, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL,
                                52, 340, 70, 24, kIdDeltaX, state.font);
    MakeControl(parent, L"STATIC", L"dY:", SS_LEFT, 132, 344, 26, 20, -1, state.font);
    state.hDeltaY = MakeControl(parent, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL,
                                158, 340, 70, 24, kIdDeltaY, state.font);
    MakeControl(parent, L"STATIC", L"dZ:", SS_LEFT, 238, 344, 26, 20, -1, state.font);
    state.hDeltaZ = MakeControl(parent, L"EDIT", L"0", WS_BORDER | ES_AUTOHSCROLL,
                                264, 340, 70, 24, kIdDeltaZ, state.font);
    MakeControl(parent, L"STATIC", L"мм, смещение от стартовой позы", SS_LEFTNOWORDWRAP,
                344, 344, 160, 20, -1, state.font);
    MakeControl(parent, L"BUTTON", L"Отправить точку", BS_PUSHBUTTON, 24, 374, 170, 30, kIdSendPoint, state.font);
    MakeControl(parent, L"BUTTON", L"Home (0, 0, 0)", BS_PUSHBUTTON, 204, 374, 170, 30, kIdHome, state.font);

    MakeControl(parent, L"BUTTON", L"Журнал", BS_GROUPBOX, 12, 438, 500, 222, -1, state.fontBold);
    state.hLog = MakeControl(parent, L"EDIT", L"",
                             WS_BORDER | ES_MULTILINE | ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
                             24, 462, 476, 188, kIdLog, state.font);

    MakeControl(parent, L"BUTTON", L"Состояние EGM", BS_GROUPBOX, 524, 10, 464, 300, -1, state.fontBold);
    state.hStatus = MakeControl(parent, L"EDIT", L"не запущен",
                                WS_BORDER | ES_MULTILINE | ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
                                538, 34, 436, 264, kIdStatus, state.font);

    MakeControl(parent, L"BUTTON", L"Порядок работы", BS_GROUPBOX, 524, 318, 464, 342, -1, state.fontBold);
    state.hWork = MakeControl(parent, L"EDIT", L"",
                              WS_BORDER | ES_MULTILINE | ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
                              538, 342, 436, 306, -1, state.font);
    ::SetWindowTextW(state.hWork,
                L"1. Загрузите файл траектории (кнопка «Открыть файл…»).\r\n"
                L"2. При необходимости сохраните точки в CSV.\r\n"
                L"3. В RobotStudio создайте систему с опцией 689-1 EGM и запустите контроллер.\r\n"
                L"4. Загрузите RAPID-модуль ModuleEGM.mod (Add Module → File…).\r\n"
                L"5. Нажмите «Запустить траекторию» — приложение займёт UDP-порт 6510\r\n"
                L"   и будет ждать первый пакет от контроллера.\r\n"
                L"6. В RobotStudio нажмите Start (вкладка RAPID).\r\n"
                L"7. Робот должен поехать в первую точку и далее по траектории.\r\n\r\n"
                L"Важно: модуль EGM настроен на базовую систему координат, поэтому\r\n"
                L"приложение отправляет АБСОЛЮТНЫЕ координаты TCP (мм).\r\n"
                L"Первая точка файла должна совпадать с home робота (MoveJ home).\r\n\r\n"
                L"Ручные команды dX/dY/dZ — то же смещение в мм от стартовой позы.\r\n\r\n"
                L"Если пакеты идут, а робот стоит: посмотрите MCI, моторы и сходимость\r\n"
                L"в панели «Состояние EGM», а также совпадает ли home с первой точкой.");
}

void AddLog(WindowState& state, const std::wstring& text)
{
    if (state.hLog == nullptr)
    {
        return;
    }

    SYSTEMTIME now{};
    ::GetLocalTime(&now);
    const std::wstring line = Format(L"%02d:%02d:%02d  ", now.wHour, now.wMinute, now.wSecond) + text + L"\r\n";

    // Не даём журналу расти бесконечно.
    if (::GetWindowTextLengthW(state.hLog) > 60000)
    {
        ::SendMessageW(state.hLog, EM_SETSEL, 0, 20000);
        ::SendMessageW(state.hLog, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(L""));
    }

    const LRESULT length = ::SendMessageW(state.hLog, WM_GETTEXTLENGTH, 0, 0);
    ::SendMessageW(state.hLog, EM_SETSEL, static_cast<WPARAM>(length), static_cast<LPARAM>(length));
    ::SendMessageW(state.hLog, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(line.c_str()));
    ::SendMessageW(state.hLog, EM_SCROLLCARET, 0, 0);
}

std::wstring MciText(int state)
{
    switch (state)
    {
    case 1: return L"ошибка";
    case 2: return L"остановлен";
    case 3: return L"работает";
    default: return L"—";
    }
}

std::wstring MotorText(int state)
{
    switch (state)
    {
    case 1: return L"включены";
    case 2: return L"выключены";
    default: return L"—";
    }
}

std::wstring BuildStatusText(const EgmSnapshot& snapshot)
{
    std::wstring text;
    text += L"Состояние: " + std::wstring(EgmStateText(snapshot.state)) + L"\r\n\r\n";
    text += Format(L"Точка: %d / %d\r\n", snapshot.pointIndex, snapshot.pointCount);
    text += L"Цель (TCP, мм): " + FormatPoint(snapshot.target) + L"\r\n";
    if (snapshot.hasFeedback)
    {
        text += L"Факт. поза (мм):       " + FormatPoint(snapshot.feedback) + L"\r\n";
    }

    text += Format(L"\r\nОтправлено пакетов: %llu\r\nПринято от робота: %llu\r\n",
                   snapshot.sentPackets, snapshot.robotPackets);
    text += L"\r\nMCI: " + MciText(snapshot.mciState) + L"\r\n";
    text += L"Моторы: " + MotorText(snapshot.motorState) + L"\r\n";
    text += L"Сходимость: " + std::wstring(snapshot.convergence ? L"да" : L"нет") + L"\r\n";

    if (!snapshot.message.empty())
    {
        text += L"\r\n" + snapshot.message;
    }
    return text;
}

int ReadInt(HWND edit, int fallback)
{
    wchar_t buffer[64] = {};
    ::GetWindowTextW(edit, buffer, 64);
    wchar_t* end = nullptr;
    const long value = wcstol(buffer, &end, 10);
    if (end == buffer)
    {
        return fallback;
    }
    return static_cast<int>(value);
}

double ReadDouble(HWND edit, double fallback)
{
    wchar_t buffer[64] = {};
    ::GetWindowTextW(edit, buffer, 64);
    wchar_t* end = nullptr;
    const double value = wcstod(buffer, &end);
    if (end == buffer)
    {
        return fallback;
    }
    return value;
}

bool BrowseFile(HWND owner, bool save, const wchar_t* filter, std::wstring& path)
{
    wchar_t buffer[1024] = {};
    if (!path.empty() && path.size() < 1000)
    {
        wcscpy_s(buffer, path.c_str());
    }

    OPENFILENAMEW dialog{};
    dialog.lStructSize = sizeof(dialog);
    dialog.hwndOwner = owner;
    dialog.lpstrFilter = filter;
    dialog.lpstrFile = buffer;
    dialog.nMaxFile = 1024;
    dialog.lpstrDefExt = save ? L"csv" : L"gcode";
    dialog.Flags = OFN_PATHMUSTEXIST | (save ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);

    const BOOL ok = save ? ::GetSaveFileNameW(&dialog) : ::GetOpenFileNameW(&dialog);
    if (ok == FALSE)
    {
        return false;
    }

    path = buffer;
    return true;
}

void UpdateInfoLabels(WindowState& state)
{
    if (state.hFileInfo == nullptr || state.hPointInfo == nullptr)
    {
        return;
    }

    std::wstring name = state.trajectoryPath;
    const size_t slash = name.find_last_of(L"\\/");
    if (slash != std::wstring::npos)
    {
        name = name.substr(slash + 1);
    }
    if (name.empty())
    {
        name = L"файл не загружен";
    }

    ::SetWindowTextW(state.hFileInfo, name.c_str());
    ::SetWindowTextW(state.hPointInfo, Format(L"точек: %d", static_cast<int>(state.points.size())).c_str());
}

void LoadTrajectoryIntoState(WindowState& state, const std::wstring& path)
{
    const TrajectoryLoadResult result = LoadTrajectory(path);
    if (!result.ok)
    {
        AddLog(state, L"не удалось загрузить траекторию: " + result.error);
        ::MessageBoxW(state.hwnd, result.error.c_str(), L"Ошибка чтения траектории", MB_ICONWARNING);
        return;
    }

    state.points = result.points;
    state.trajectoryPath = path;
    UpdateInfoLabels(state);

    const wchar_t* format = (result.detected == TrajectoryFormat::GCode) ? L"G-code" : L"простой XYZ";
    AddLog(state, Format(L"загружено точек: %d (формат: %s)",
                         static_cast<int>(state.points.size()), format));
    if (!result.warnings.empty())
    {
        AddLog(state, L"предупреждения: " + result.warnings);
    }
}

void LoadDefaultTrajectory(WindowState& state)
{
    const std::wstring defaultPath = Utf8ToWide(FREZA_DATA_DIR) + L"/trajectory.gcode";
    LoadTrajectoryIntoState(state, defaultPath);
}

EgmSettings ReadSettings(WindowState& state)
{
    EgmSettings settings;
    settings.port = ReadInt(state.hPort, 6510);
    settings.dwellMs = 1000; // пауза только для одиночных ручных команд
    settings.sendPeriodMs = ReadInt(state.hPeriod, 4);
    return settings;
}

void StartStream(WindowState& state, const std::vector<Point3>& deltas,
                 const std::vector<int>& dwells,
                 const std::vector<unsigned char>& corners,
                 const std::wstring& what)
{
    std::wstring error;
    if (!state.egm.Start(ReadSettings(state), deltas, dwells, corners, error))
    {
        AddLog(state, L"запуск не удался: " + error);
        ::MessageBoxW(state.hwnd, error.c_str(), L"EGM", MB_ICONWARNING);
        return;
    }
    AddLog(state, Format(L"запущено (%s): точек %d", what.c_str(), static_cast<int>(deltas.size())));
}

void StartStream(WindowState& state, const std::vector<Point3>& deltas,
                 const std::vector<int>& dwells, const std::wstring& what)
{
    StartStream(state, deltas, dwells, std::vector<unsigned char>(), what);
}

void StartStream(WindowState& state, const std::vector<Point3>& deltas, const std::wstring& what)
{
    StartStream(state, deltas, std::vector<int>(), what);
}

void OnOpenTrajectory(WindowState& state)
{
    std::wstring path;
    if (!BrowseFile(state.hwnd, false, kFilterTrajectory, path))
    {
        return;
    }
    LoadTrajectoryIntoState(state, path);
}

void OnSaveCsv(WindowState& state)
{
    if (state.points.empty())
    {
        AddLog(state, L"нечего сохранять: траектория не загружена");
        return;
    }

    std::wstring path = state.trajectoryPath;
    if (!path.empty())
    {
        const size_t dot = path.find_last_of(L'.');
        const size_t slash = path.find_last_of(L"\\/");
        if (dot != std::wstring::npos && (slash == std::wstring::npos || dot > slash))
        {
            path = path.substr(0, dot);
        }
        path += L".csv";
    }
    else
    {
        path = Utf8ToWide(FREZA_DATA_DIR) + L"/points.csv";
    }

    if (!BrowseFile(state.hwnd, true, kFilterCsv, path))
    {
        return;
    }

    std::wstring error;
    if (!WritePointsCsv(path, state.points, error))
    {
        AddLog(state, L"ошибка записи CSV: " + error);
        ::MessageBoxW(state.hwnd, error.c_str(), L"CSV", MB_ICONWARNING);
        return;
    }
    AddLog(state, Format(L"CSV сохранён: точек %d, файл %s",
                         static_cast<int>(state.points.size()), path.c_str()));
}

void OnLoadCsv(WindowState& state)
{
    std::wstring path;
    if (!BrowseFile(state.hwnd, false, kFilterCsv, path))
    {
        return;
    }

    std::vector<Point3> points;
    std::wstring error;
    if (!ReadPointsCsv(path, points, error))
    {
        AddLog(state, L"ошибка чтения CSV: " + error);
        ::MessageBoxW(state.hwnd, error.c_str(), L"CSV", MB_ICONWARNING);
        return;
    }

    state.points = std::move(points);
    state.trajectoryPath = path;
    UpdateInfoLabels(state);
    AddLog(state, Format(L"CSV загружен: точек %d", static_cast<int>(state.points.size())));
}

void OnStartTrajectory(WindowState& state)
{
    if (state.points.size() < 2)
    {
        AddLog(state, L"для запуска нужна траектория минимум из двух точек");
        return;
    }

    // EGM в этом модуле настроен на базовую систему координат (corr_frame =
    // [[0,0,0],[1,0,0,0]], EGM_FRAME_BASE), поэтому отправляются АБСОЛЮТНЫЕ
    // координаты TCP, а не смещения от home.
    const std::vector<Point3>& path = state.points;

    const bool interpolate = ::SendMessageW(state.hInterp, BM_GETCHECK, 0, 0) == BST_CHECKED;
    if (!interpolate)
    {
        StartStream(state, path, L"траектория");
        return;
    }

    const double step = ReadDouble(state.hStep, 2.0);
    const double feed = ReadDouble(state.hFeed, 50.0);

    std::vector<Point3> dense;
    std::vector<int> dwells;
    std::vector<unsigned char> corners;
    InterpolatePath(path, step, feed, dense, dwells, corners);
    if (dense.size() < 2)
    {
        AddLog(state, L"интерполяция не дала точек — проверьте шаг и траекторию");
        return;
    }

    AddLog(state, Format(L"интерполяция: точек %d, шаг %.1f мм, скорость %.1f мм/с",
                         static_cast<int>(dense.size()), step, feed));
    StartStream(state, dense, dwells, corners, L"траектория (интерполяция)");
}

// Базовая точка для ручных команд: первая точка загруженной траектории (она же
// совпадает с home в RAPID).
bool TryGetBasePoint(const WindowState& state, Point3& base)
{
    if (state.points.empty())
    {
        return false;
    }
    base = state.points.front();
    return true;
}

void OnStop(WindowState& state)
{
    state.egm.Stop();
    AddLog(state, L"остановлено пользователем");
}

void OnSendManualPoint(WindowState& state)
{
    Point3 base{};
    if (!TryGetBasePoint(state, base))
    {
        AddLog(state, L"сначала загрузите траекторию: нужна стартовая точка для отсчёта");
        return;
    }

    const Point3 delta{ReadDouble(state.hDeltaX, 0.0),
                       ReadDouble(state.hDeltaY, 0.0),
                       ReadDouble(state.hDeltaZ, 0.0)};
    const Point3 target{base.x + delta.x, base.y + delta.y, base.z + delta.z};

    StartStream(state, std::vector<Point3>{target},
                Format(L"ручная команда %s (от старта)", FormatPoint(delta).c_str()));
}

void OnHome(WindowState& state)
{
    Point3 base{};
    if (!TryGetBasePoint(state, base))
    {
        AddLog(state, L"сначала загрузите траекторию: неизвестна стартовая точка");
        return;
    }
    StartStream(state, std::vector<Point3>{base}, L"возврат в Home");
}

void RefreshStatus(WindowState& state)
{
    const EgmSnapshot snapshot = state.egm.Snapshot();
    ::SetWindowTextW(state.hStatus, BuildStatusText(snapshot).c_str());

    for (const std::wstring& line : state.egm.TakeLog())
    {
        AddLog(state, line);
    }

    ::EnableWindow(state.hStop, state.egm.Running() ? TRUE : FALSE);
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

WindowState* GetState(HWND hwnd)
{
    return reinterpret_cast<WindowState*>(::GetWindowLongPtrW(hwnd, GWLP_USERDATA));
}

} // namespace

bool AppWindow::Create(HINSTANCE instance, int showCommand)
{
    m_instance = instance;

    WNDCLASSEXW windowClass{};
    windowClass.cbSize = sizeof(windowClass);
    windowClass.style = CS_HREDRAW | CS_VREDRAW;
    windowClass.lpfnWndProc = WndProc;
    windowClass.hInstance = instance;
    windowClass.hCursor = ::LoadCursorW(nullptr, IDC_ARROW);
    windowClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_BTNFACE + 1);
    windowClass.lpszClassName = kClassName;

    if (::RegisterClassExW(&windowClass) == 0)
    {
        return false;
    }

    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;

    RECT rect{0, 0, 1000, 672};
    ::AdjustWindowRect(&rect, style, FALSE);

    m_hwnd = ::CreateWindowExW(0, kClassName, kTitle,
                               style,
                               CW_USEDEFAULT, CW_USEDEFAULT,
                               rect.right - rect.left, rect.bottom - rect.top,
                               nullptr, nullptr, instance, nullptr);

    if (m_hwnd == nullptr)
    {
        return false;
    }

    ::ShowWindow(m_hwnd, showCommand);
    ::UpdateWindow(m_hwnd);
    return true;
}

int AppWindow::Run()
{
    MSG message{};
    while (::GetMessageW(&message, nullptr, 0, 0) > 0)
    {
        ::TranslateMessage(&message);
        ::DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}

namespace {

LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_CREATE:
    {
        auto* state = new WindowState();
        state->hwnd = hwnd;
        state->font = CreateUiFont(9, false);
        state->fontBold = CreateUiFont(10, true);
        state->whiteBrush = ::CreateSolidBrush(RGB(255, 255, 255));
        CreateControls(*state);
        ::SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(state));
        ::SetTimer(hwnd, 1, 100, nullptr);
        LoadDefaultTrajectory(*state);
        return 0;
    }

    case WM_TIMER:
    {
        WindowState* state = GetState(hwnd);
        if (state != nullptr && wParam == 1)
        {
            RefreshStatus(*state);
        }
        return 0;
    }

    case WM_COMMAND:
    {
        WindowState* state = GetState(hwnd);
        if (state == nullptr)
        {
            break;
        }

        switch (LOWORD(wParam))
        {
        case kIdOpenTrajectory: OnOpenTrajectory(*state); return 0;
        case kIdSaveCsv:        OnSaveCsv(*state); return 0;
        case kIdLoadCsv:        OnLoadCsv(*state); return 0;
        case kIdStart:          OnStartTrajectory(*state); return 0;
        case kIdStop:           OnStop(*state); return 0;
        case kIdSendPoint:      OnSendManualPoint(*state); return 0;
        case kIdHome:           OnHome(*state); return 0;
        default:                break;
        }
        break;
    }

    case WM_CTLCOLORSTATIC:
    {
        // Белый фон для наших read-only многострочных полей (статус, справка, журнал).
        WindowState* state = GetState(hwnd);
        const HWND control = reinterpret_cast<HWND>(lParam);
        if (state != nullptr && state->whiteBrush != nullptr &&
            (control == state->hStatus || control == state->hWork || control == state->hLog))
        {
            ::SetBkColor(reinterpret_cast<HDC>(wParam), RGB(255, 255, 255));
            return reinterpret_cast<LRESULT>(state->whiteBrush);
        }
        break;
    }

    case WM_HSCROLL:
    {
        WindowState* state = GetState(hwnd);
        if (state != nullptr && reinterpret_cast<HWND>(lParam) == state->hSpeed)
        {
            const int value = static_cast<int>(::SendMessageW(state->hSpeed, TBM_GETPOS, 0, 0));
            ::SetWindowTextW(state->hFeed, std::to_wstring(value).c_str());
        }
        return 0;
    }

    case WM_CLOSE:
        ::DestroyWindow(hwnd);
        return 0;

    case WM_DESTROY:
    {
        ::KillTimer(hwnd, 1);
        WindowState* state = GetState(hwnd);
        if (state != nullptr)
        {
            state->egm.Stop();
            ::SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
            if (state->font != nullptr)
            {
                ::DeleteObject(state->font);
            }
            if (state->fontBold != nullptr)
            {
                ::DeleteObject(state->fontBold);
            }
            if (state->whiteBrush != nullptr)
            {
                ::DeleteObject(state->whiteBrush);
            }
            delete state;
        }
        ::PostQuitMessage(0);
        return 0;
    }

    default:
        break;
    }

    (void)lParam;
    return ::DefWindowProcW(hwnd, message, wParam, lParam);
}

} // namespace
} // namespace freza
