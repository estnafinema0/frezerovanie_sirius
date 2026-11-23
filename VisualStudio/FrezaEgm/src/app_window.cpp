#include "app_window.h"

#include "csv_io.h"
#include "cam.h"
#include <shellapi.h>
#include <cmath>
#include <cwctype>
#include <limits>
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

// STL is listed here on purpose: selecting an STL in this dialog must go through
// the CAM path, not through the G-code/XYZ trajectory parser.
constexpr wchar_t kFilterTrajectory[] =
    L"Траектория (*.gcode;*.nc;*.ncl;*.xyz;*.txt) или STL (*.stl)\0*.gcode;*.nc;*.ncl;*.xyz;*.txt;*.stl\0"
    L"Траектория (*.gcode;*.nc;*.ncl;*.xyz;*.txt)\0*.gcode;*.nc;*.ncl;*.xyz;*.txt\0"
    L"STL для CAM (*.stl)\0*.stl\0Все файлы (*.*)\0*.*\0\0";
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
    kIdStl = 1050, kIdDemo = 1051, kIdUnits = 1052, kIdFace = 1053,
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

    HWND hUnits = nullptr, hScale = nullptr, hDiameter = nullptr, hCamStep = nullptr;
    HWND hFace = nullptr;
    HWND hTx = nullptr, hTy = nullptr, hTz = nullptr;
    std::unique_ptr<cam::Result> camResult;
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

    MakeControl(parent, L"BUTTON", L"STL / OpenCAMLib — до запуска EGM", BS_GROUPBOX,
                524, 318, 464, 342, -1, state.fontBold);
    MakeControl(parent, L"STATIC", L"Единицы STL:", SS_LEFT, 536, 342, 94, 20, -1, state.font);
    state.hUnits = MakeControl(parent, L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_VSCROLL,
                               632, 338, 90, 100, kIdUnits, state.font);
    ::SendMessageW(state.hUnits, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"мм"));
    ::SendMessageW(state.hUnits, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"дюймы"));
    MakeControl(parent, L"STATIC", L"масштаб:", SS_LEFT, 738, 342, 65, 20, -1, state.font);
    state.hScale = MakeControl(parent, L"EDIT", L"1", WS_BORDER, 810, 338, 70, 24, -1, state.font);
    MakeControl(parent, L"STATIC", L"Фреза Ø, мм:", SS_LEFT, 536, 374, 94, 20, -1, state.font);
    state.hDiameter = MakeControl(parent, L"EDIT", L"4", WS_BORDER, 632, 370, 70, 24, -1, state.font);
    MakeControl(parent, L"STATIC", L"шаг CAM, мм:", SS_LEFT, 720, 374, 95, 20, -1, state.font);
    state.hCamStep = MakeControl(parent, L"EDIT", L"0.5", WS_BORDER, 820, 370, 60, 24, -1, state.font);
    MakeControl(parent, L"STATIC", L"Перенос XYZ в базу, мм (без поворота):", SS_LEFT, 536, 403, 420, 20, -1, state.font);
    state.hTx = MakeControl(parent, L"EDIT", L"", WS_BORDER, 536, 426, 85, 24, -1, state.font);
    state.hTy = MakeControl(parent, L"EDIT", L"", WS_BORDER, 630, 426, 85, 24, -1, state.font);
    state.hTz = MakeControl(parent, L"EDIT", L"", WS_BORDER, 724, 426, 85, 24, -1, state.font);
    MakeControl(parent, L"BUTTON", L"Демо 20×20", BS_PUSHBUTTON, 820, 424, 150, 28, kIdDemo, state.font);
    MakeControl(parent, L"BUTTON", L"STL → рассчитать…", BS_PUSHBUTTON, 536, 460, 180, 28, kIdStl, state.font);
    state.hFace = MakeControl(parent, L"BUTTON", L"Вся верхняя поверхность (змейка)", BS_AUTOCHECKBOX,
                              724, 462, 250, 24, kIdFace, state.font);
    state.hWork = MakeControl(parent, L"EDIT", L"«Демо 20×20» — пример размещения, не калибровка стенда.\r\n"
        L"Без галочки — проход по периметру верхней грани.\r\n"
        L"С галочкой — змейка по всей верхней грани, шаг 0.7 × Ø фрезы.\r\n"
        L"Только прямоугольный блок: 12 треугольников.\r\n"
        L"Расчёт не запускает движение. Экспорт: Сохранить CSV.\r\n"
        L"Длина CylCutter = 20 мм; к TCP не прибавляется.",
        WS_BORDER | ES_MULTILINE | ES_READONLY | WS_VSCROLL, 536, 496, 438, 152, -1, state.font);

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
    if (state.egm.Running()) { AddLog(state, L"Сначала остановите EGM: источник траектории занят."); return; }
    const TrajectoryLoadResult result = LoadTrajectory(path);
    if (!result.ok)
    {
        AddLog(state, L"не удалось загрузить траекторию: " + result.error);
        ::MessageBoxW(state.hwnd, result.error.c_str(), L"Ошибка чтения траектории", MB_ICONWARNING);
        return;
    }

    state.camResult.reset();
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

    // Точность углов. Значение по умолчанию 1.5 мм означало «угол достигнут,
    // когда TCP ещё в 1.5 мм от вершины»: цель переключалась на следующую
    // сторону раньше времени, и квадрат получался со срезанными углами.
    // Теперь на вершине ждём фактического совпадения (0.1 мм), а не 1.5 мм;
    // верхняя граница ожидания поднята, чтобы успеть доехать на высокой подаче.
    settings.reachToleranceMm = 0.1;
    settings.cornerSettleMs = 2000;
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
    if (!corners.empty())
    {
        AddLog(state, L"углы: цель удерживается на вершине (остановка), допуск достижения 0.1 мм, до 2000 мс");
    }
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

bool IsStlPath(const std::wstring& path)
{
    if (path.empty())
    {
        return false;
    }
    const std::wstring extension = std::filesystem::path(path).extension().wstring();
    return _wcsicmp(extension.c_str(), L".stl") == 0;
}

void OnStl(WindowState& state, std::wstring path = {});

void OnOpenTrajectory(WindowState& state)
{
    std::wstring path;
    if (!BrowseFile(state.hwnd, false, kFilterTrajectory, path))
    {
        return;
    }
    // An STL is a mesh, not a trajectory: it must be converted by CAM. Parsing it
    // as G-code/XYZ used to end with "меньше двух точек траектории".
    if (IsStlPath(path))
    {
        OnStl(state, path);
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
    if (state.egm.Running()) { AddLog(state, L"Сначала остановите EGM: источник траектории занят."); return; }
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

    state.camResult.reset();
    state.points = std::move(points);
    state.trajectoryPath = path;
    UpdateInfoLabels(state);
    AddLog(state, Format(L"CSV загружен: точек %d", static_cast<int>(state.points.size())));
}

// Strict numeric entry is confined to the new CAM fields.
double CamNumber(HWND edit)
{
    wchar_t text[128] = {};
    ::GetWindowTextW(edit, text, 128);
    wchar_t* end = nullptr;
    const double v = wcstod(text, &end);
    const bool parsed = end != text;
    while (end && iswspace(*end)) ++end;
    return !parsed || !end || *end || !std::isfinite(v)
        ? std::numeric_limits<double>::quiet_NaN() : v;
}

void OnStl(WindowState& state, std::wstring path)
{
    if (state.egm.Running()) { AddLog(state, L"Сначала остановите EGM: импорт STL заблокирован."); return; }
    if (!cam::Available()) {
        AddLog(state, L"STL: сборка без CAM (FREZA_ENABLE_CAM=OFF).");
        ::MessageBoxW(state.hwnd, L"Эта сборка сделана без STL/CAM. Включите FREZA_ENABLE_CAM и соберите OpenCAMLib "
            L"(docs/13-stl-cam.md, preset x64-cam-release). Загрузка координат и CSV в этой сборке работает.",
            L"STL", MB_ICONINFORMATION);
        return;
    }
    cam::Options options;
    options.strategy = (::SendMessageW(state.hFace, BM_GETCHECK, 0, 0) == BST_CHECKED)
        ? cam::Strategy::FaceZigzag : cam::Strategy::Perimeter;
    auto units = ::SendMessageW(state.hUnits, CB_GETCURSEL, 0, 0);
    if (units != 0 && units != 1)
    {
        // STL has no units of its own. Ask explicitly instead of silently guessing.
        const int answer = ::MessageBoxW(state.hwnd,
            L"Единицы STL не выбраны.\r\n\r\nСчитать размеры модели в миллиметрах?\r\n\r\n"
            L"«Нет» — прервать и выбрать «мм» или «дюймы» в блоке «STL / OpenCAMLib».",
            L"STL: единицы модели", MB_YESNO | MB_ICONQUESTION);
        if (answer != IDYES)
        {
            ::SetFocus(state.hUnits);
            AddLog(state, L"STL: выберите единицы (мм или дюймы) и повторите расчёт.");
            return;
        }
        ::SendMessageW(state.hUnits, CB_SETCURSEL, 0, 0);
        units = 0;
    }
    options.units = units == 0 ? cam::Units::Millimetres : cam::Units::Inches;
    options.scale = CamNumber(state.hScale);
    options.diameterMm = CamNumber(state.hDiameter);
    options.samplingMm = CamNumber(state.hCamStep);
    if (!std::isfinite(CamNumber(state.hTx)) || !std::isfinite(CamNumber(state.hTy)) ||
        !std::isfinite(CamNumber(state.hTz)))
    {
        // Base coordinates need an explicit part placement; offer the documented demo one.
        const int answer = ::MessageBoxW(state.hwnd,
            L"Перенос XYZ в базу не задан.\r\n\r\nПодставить демонстрационное размещение X=750, Y=0, Z=599.5?\r\n\r\n"
            L"«Нет» — ввести X/Y/Z вручную в блоке «STL / OpenCAMLib».",
            L"STL: размещение детали", MB_YESNO | MB_ICONQUESTION);
        if (answer != IDYES)
        {
            ::SetFocus(state.hTx);
            AddLog(state, L"STL: задайте перенос XYZ в базу и повторите расчёт.");
            return;
        }
        ::SetWindowTextW(state.hTx, L"750");
        ::SetWindowTextW(state.hTy, L"0");
        ::SetWindowTextW(state.hTz, L"599.5");
        AddLog(state, L"STL: подставлено демо-размещение [750, 0, 599.5]. Это не калибровка стенда.");
    }
    options.translationMm = {CamNumber(state.hTx), CamNumber(state.hTy), CamNumber(state.hTz)};
    if (path.empty() && !BrowseFile(state.hwnd, false, L"STL (*.stl)\0*.stl\0\0", path)) return;
    auto result = std::make_unique<cam::Result>();
    std::string error;
    ::SetCursor(::LoadCursorW(nullptr, IDC_WAIT));
    const bool ok = cam::Calculate(std::filesystem::path(path), options, *result, error);
    ::SetCursor(::LoadCursorW(nullptr, IDC_ARROW));
    if (!ok) {
        const std::wstring reason = Utf8ToWide(error);
        AddLog(state, L"STL: " + reason);
        const std::wstring hint = reason + L"\r\n\r\nПроверьте единицы (мм/дюймы), масштаб, перенос XYZ, Ø фрезы и шаг CAM.\r\n"
            L"«Демо 20×20» подставляет пример размещения для Primer1.STL.\r\n"
            L"Поддерживается только замкнутый прямоугольный блок из 12 треугольников.";
        ::MessageBoxW(state.hwnd, hint.c_str(), L"STL / CAM", MB_ICONWARNING);
        return;
    }
    const std::wstring summary = Utf8ToWide(cam::Describe(*result));
    const std::wstring preview = summary + L"\r\n\r\nРазмещение задаётся пользователем; это не измеренная калибровка."
        L"\r\nПроверьте границы. OK заменит траекторию; движение запускается отдельно.";
    if (::MessageBoxW(state.hwnd, preview.c_str(), L"Проверка STL → TCP в базе", MB_OKCANCEL | MB_ICONINFORMATION) != IDOK) return;
    if (state.egm.Running()) return;
    state.points = result->basePoints;
    state.camResult = std::move(result);
    state.trajectoryPath = path;
    ::SetWindowTextW(state.hWork, summary.c_str());
    UpdateInfoLabels(state);
    AddLog(state, Format(L"STL: расчёт и проверка завершены (%s): точек %d, контактов %d; EGM не запущен.",
                         state.camResult->options.strategy == cam::Strategy::FaceZigzag
                             ? L"вся верхняя поверхность"
                             : L"периметр",
                         static_cast<int>(state.camResult->basePoints.size()),
                         static_cast<int>(state.camResult->contactCount)));
}

bool CheckCamPath(WindowState& state, const std::vector<Point3>& path)
{
    if (!state.camResult) return true;
    std::string error;
    if (cam::ValidatePath(*state.camResult, path, error)) return true;
    AddLog(state, L"CAM: " + Utf8ToWide(error));
    ::MessageBoxW(state.hwnd, Utf8ToWide(error).c_str(), L"Проверка траектории", MB_ICONWARNING);
    return false;
}

void OnStartTrajectory(WindowState& state)
{
    if (state.points.size() < 2)
    {
        AddLog(state, L"для запуска нужна траектория минимум из двух точек");
        return;
    }

    if (state.camResult) {
        if (state.egm.Running()) { AddLog(state, L"Сначала остановите текущую сессию EGM."); return; }
        if (!CheckCamPath(state, state.points)) return;
        const auto first = state.points.front();
        if (std::abs(first.x-750)>1e-6 || std::abs(first.y)>1e-6 || std::abs(first.z-600)>1e-6) {
            // Первая точка CAM-пути = угол верхней грани + перенос. Чтобы она совпала
            // с home, перенос должен быть ровно (home - этот угол). Считаем его и
            // подставляем в поля, иначе оператор подбирает числа вручную.
            const auto& t = state.camResult->options.translationMm;
            const double nx = t.x + (750.0 - first.x);
            const double ny = t.y + (0.0 - first.y);
            const double nz = t.z + (600.0 - first.z);
            ::SetWindowTextW(state.hTx, Format(L"%.3f", nx).c_str());
            ::SetWindowTextW(state.hTy, Format(L"%.3f", ny).c_str());
            ::SetWindowTextW(state.hTz, Format(L"%.3f", nz).c_str());
            const std::wstring text = Format(
                L"Начало CAM-траектории (%.3f, %.3f, %.3f) не совпадает с home [750, 0, 600],\r\n"
                L"поэтому подвод от home не рассчитывается.\r\n\r\n"
                L"Нужный перенос детали: X = %.3f, Y = %.3f, Z = %.3f\r\n"
                L"Эти значения уже подставлены в блок «STL / OpenCAMLib».\r\n\r\n"
                L"Нажмите «STL → рассчитать…» и выберите тот же STL — перенос применяется\r\n"
                L"только при новом расчёте. Затем запускайте траекторию.",
                first.x, first.y, first.z, nx, ny, nz);
            AddLog(state, Format(L"CAM: первая точка (%.3f, %.3f, %.3f), нужен перенос (%.3f, %.3f, %.3f)",
                                 first.x, first.y, first.z, nx, ny, nz));
            ::MessageBoxW(state.hwnd, text.c_str(), L"CAM: перенос детали", MB_ICONWARNING);
            return;
        }
    }

    // EGM в этом модуле настроен на базовую систему координат (corr_frame =
    // [[0,0,0],[1,0,0,0]], EGM_FRAME_BASE), поэтому отправляются АБСОЛЮТНЫЕ
    // координаты TCP, а не смещения от home.
    const std::vector<Point3>& path = state.points;

    const bool interpolate = ::SendMessageW(state.hInterp, BM_GETCHECK, 0, 0) == BST_CHECKED;
    if (!interpolate)
    {
        // Без интерполяции каждая точка — вершина: держим цель, пока TCP реально не
        // доедет (и не меньше 1 с), иначе на скорости угол проскакивает со скосом.
        std::vector<int> dwells(path.size(), 1000);
        std::vector<unsigned char> corners(path.size(), 1);
        StartStream(state, path, dwells, corners, L"траектория");
        return;
    }

    const double step = ReadDouble(state.hStep, 2.0);
    const double feed = ReadDouble(state.hFeed, 50.0);

    if (state.camResult) {
        std::string error;
        if (!cam::ValidateInterpolation(*state.camResult, step, feed, error)) {
            ::MessageBoxW(state.hwnd, Utf8ToWide(error).c_str(), L"CAM / интерполяция", MB_ICONWARNING);
            return;
        }
    }
    std::vector<Point3> dense;
    std::vector<int> dwells;
    std::vector<unsigned char> corners;
    InterpolatePath(path, step, feed, dense, dwells, corners);
    if (dense.size() < 2)
    {
        AddLog(state, L"интерполяция не дала точек — проверьте шаг и траекторию");
        return;
    }

    // Углы: держим цель на вершине не меньше kCornerStopMs, чтобы TCP успел
    // физически остановиться. Проверка «дошёл до вершины» срабатывает, когда TCP
    // ещё движется, и контроллер сглаживает переход — на экране это скос угла.
    constexpr int kCornerStopMs = 500;
    for (size_t i = 0; i < corners.size() && i < dwells.size(); ++i)
    {
        if (corners[i] != 0 && dwells[i] < kCornerStopMs)
        {
            dwells[i] = kCornerStopMs;
        }
    }

    if (!CheckCamPath(state, dense)) return;

    AddLog(state, Format(L"интерполяция: точек %d, шаг %.1f мм, скорость %.1f мм/с",
                         static_cast<int>(dense.size()), step, feed));
    AddLog(state, Format(L"вершины: остановка до %.0f мс, ожидание достижения 0.1 мм", static_cast<double>(kCornerStopMs)));
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
        ::DragAcceptFiles(hwnd, TRUE);
        return 0;
    }

    case WM_DROPFILES:
    {
        auto drop = reinterpret_cast<HDROP>(wParam);
        std::wstring path;
        if (::DragQueryFileW(drop, 0xFFFFFFFF, nullptr, 0) == 1) {
            const UINT n = ::DragQueryFileW(drop, 0, nullptr, 0);
            std::vector<wchar_t> buffer(static_cast<size_t>(n)+1);
            ::DragQueryFileW(drop, 0, buffer.data(), n+1);
            path = buffer.data();
        }
        ::DragFinish(drop);
        auto* state = GetState(hwnd);
        if (state && !path.empty()) {
            if (_wcsicmp(std::filesystem::path(path).extension().c_str(), L".stl") == 0) OnStl(*state, path);
            else AddLog(*state, L"Перетаскивание: поддерживается один файл STL.");
        }
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
        case kIdStl:            OnStl(*state); return 0;
        case kIdDemo:
            ::SendMessageW(state->hUnits, CB_SETCURSEL, 0, 0);
            ::SetWindowTextW(state->hScale, L"1");
            ::SetWindowTextW(state->hTx, L"750");
            ::SetWindowTextW(state->hTy, L"0");
            ::SetWindowTextW(state->hTz, L"599.5");
            AddLog(*state, L"Демо-размещение пластины 20×20×0.5: перенос [750,0,599.5]. Это не калибровка стенда.");
            return 0;
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
