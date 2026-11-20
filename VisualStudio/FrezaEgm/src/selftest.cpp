// selftest.cpp — автономная проверка без RobotStudio.
//
// Проверяет всю цепочку, кроме реального робота:
//   файл траектории -> точки -> CSV -> обратное чтение -> интерполяция
//   -> protobuf-сообщение -> UDP-обмен с «заглушкой контроллера».
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include "csv_io.h"
#include "egm3.pb.h"
#include "egm_client.h"
#include "pb_decode.h"
#include "text_io.h"
#include "trajectory.h"

#include <fcntl.h>
#include <io.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace {

int g_failures = 0;

void Check(bool condition, const wchar_t* what)
{
    wprintf(condition ? L"[ OK ] %s\n" : L"[FAIL] %s\n", what);
    if (!condition)
    {
        ++g_failures;
    }
}

bool NearlyEqual(double a, double b)
{
    return std::fabs(a - b) < 1e-6;
}

bool CheckPoint(const freza::Point3& point, double x, double y, double z)
{
    return NearlyEqual(point.x, x) && NearlyEqual(point.y, y) && NearlyEqual(point.z, z);
}

// Проверка сериализации: смещение (10, 0, 0) должно уйти вместе с явными нулями.
void TestSerialization()
{
    unsigned char buffer[256] = {};
    const size_t size = freza::EncodeEgmSensor(freza::Point3{10.0, 0.0, 0.0}, 42, 1000, buffer, sizeof(buffer));
    Check(size > 0, L"EgmSensor сериализуется");

    wprintf(L"       байты (%zu): ", size);
    for (size_t i = 0; i < size; ++i)
    {
        wprintf(L"%02X ", buffer[i]);
    }
    wprintf(L"\n");

    EgmSensor parsed = EgmSensor_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(buffer, size);
    const bool decoded = pb_decode(&stream, EgmSensor_fields, &parsed);
    Check(decoded, L"EgmSensor разбирается обратно");

    Check(parsed.has_header && parsed.header.has_seqno && parsed.header.seqno == 42,
          L"seqno передаётся");
    Check(parsed.has_header && parsed.header.mtype == EgmHeader_MessageType_MSGTYPE_CORRECTION,
          L"mtype = MSGTYPE_CORRECTION");
    Check(parsed.has_planned && parsed.planned.has_cartesian && parsed.planned.cartesian.has_pos,
          L"planned.cartesian.pos присутствует");
    Check(NearlyEqual(parsed.planned.cartesian.pos.x, 10.0), L"x = 10");
    Check(parsed.planned.cartesian.pos.has_y && NearlyEqual(parsed.planned.cartesian.pos.y, 0.0),
          L"y = 0 передаётся явно (важно для proto2-парсера ABB)");
    Check(parsed.planned.cartesian.pos.has_z && NearlyEqual(parsed.planned.cartesian.pos.z, 0.0),
          L"z = 0 передаётся явно");
    Check(parsed.planned.cartesian.has_orient && parsed.planned.cartesian.orient.has_u0 &&
              NearlyEqual(parsed.planned.cartesian.orient.u0, 1.0),
          L"ориентация передаётся (единичный кватернион)");
}

// Заглушка контроллера: шлём EgmRobot, принимаем EgmSensor, проверяем содержимое.
void TestUdpLoopback()
{
    using Clock = std::chrono::steady_clock;

    const int port = 6510;
    SOCKET robot = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    Check(robot != INVALID_SOCKET, L"создан сокет-заглушка контроллера");
    if (robot == INVALID_SOCKET)
    {
        return;
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local.sin_port = 0;
    if (::bind(robot, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == SOCKET_ERROR)
    {
        Check(false, L"сокет-заглушка занял порт");
        ::closesocket(robot);
        return;
    }

    u_long nonBlocking = 1;
    ::ioctlsocket(robot, FIONBIO, &nonBlocking);

    sockaddr_in client{};
    client.sin_family = AF_INET;
    client.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    client.sin_port = htons(static_cast<u_short>(port));

    freza::EgmClient egm;
    freza::EgmSettings settings;
    settings.port = port;
    settings.dwellMs = 200;
    settings.sendPeriodMs = 5;

    const std::vector<freza::Point3> targets{
        freza::Point3{0.0, 0.0, 0.0},
        freza::Point3{10.0, 0.0, 0.0},
        freza::Point3{10.0, 10.0, 0.0},
    };

    std::wstring error;
    const bool started = egm.Start(settings, targets, error);
    Check(started, L"EGM-клиент занял UDP-порт 6510");
    if (!started)
    {
        ::closesocket(robot);
        return;
    }

    int responses = 0;
    bool sawOrigin = false;
    bool sawTen = false;

    const Clock::time_point startedAt = Clock::now();
    uint32_t seqno = 0;
    while (Clock::now() - startedAt < std::chrono::milliseconds(1200))
    {
        unsigned char tx[256] = {};
        const size_t size = freza::EncodeEgmRobotTest(seqno++, tx, sizeof(tx));
        if (size > 0)
        {
            ::sendto(robot, reinterpret_cast<const char*>(tx), static_cast<int>(size), 0,
                     reinterpret_cast<const sockaddr*>(&client), sizeof(client));
        }

        unsigned char rx[512] = {};
        for (;;)
        {
            sockaddr_in from{};
            int fromLength = static_cast<int>(sizeof(from));
            const int received = ::recvfrom(robot, reinterpret_cast<char*>(rx), static_cast<int>(sizeof(rx)), 0,
                                            reinterpret_cast<sockaddr*>(&from), &fromLength);
            if (received <= 0)
            {
                break;
            }

            ++responses;

            EgmSensor sensor = EgmSensor_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(rx, static_cast<size_t>(received));
            if (!pb_decode(&stream, EgmSensor_fields, &sensor))
            {
                continue;
            }
            if (sensor.has_planned && sensor.planned.has_cartesian && sensor.planned.cartesian.has_pos)
            {
                const EgmCartesian& pos = sensor.planned.cartesian.pos;
                if (NearlyEqual(pos.x, 0.0) && NearlyEqual(pos.y, 0.0) && NearlyEqual(pos.z, 0.0))
                {
                    sawOrigin = true;
                }
                if (NearlyEqual(pos.x, 10.0) && NearlyEqual(pos.y, 0.0))
                {
                    sawTen = true;
                }
            }
        }

        ::Sleep(5);
    }

    const freza::EgmSnapshot snapshot = egm.Snapshot();
    egm.Stop();
    ::closesocket(robot);

    Check(responses > 20, L"клиент прислал десятки EgmSensor (приём работает)");
    Check(snapshot.robotPackets > 20, L"клиент принял EgmRobot (обратная связь работает)");
    Check(snapshot.sentPackets > 20, L"клиент отправил EgmSensor");
    Check(sawOrigin, L"первая точка — смещение 0,0,0");
    Check(sawTen, L"вторая точка — смещение 10,0,0");
}

} // namespace

int wmain(int argc, wchar_t** argv)
{
    _setmode(_fileno(stdout), _O_U8TEXT);

    WSADATA wsa{};
    if (::WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        wprintf(L"Не удалось инициализировать Winsock.\n");
        return 3;
    }

    std::wstring trajectoryPath = (argc > 1) ? argv[1] : (freza::Utf8ToWide(FREZA_DATA_DIR) + L"/trajectory.gcode");

    // Куда писать CSV: явный аргумент, затем %TEMP%, затем каталог траектории, затем cwd.
    std::vector<std::wstring> csvCandidates;
    if (argc > 2)
    {
        csvCandidates.push_back(argv[2]);
    }
    else
    {
        wchar_t tempDir[MAX_PATH] = {};
        if (::GetTempPathW(MAX_PATH, tempDir) > 0)
        {
            csvCandidates.push_back(std::wstring(tempDir) + L"freza_points_selftest.csv");
        }
        const size_t slash = trajectoryPath.find_last_of(L"\\/");
        if (slash != std::wstring::npos)
        {
            csvCandidates.push_back(trajectoryPath.substr(0, slash + 1) + L"points_selftest.csv");
        }
        csvCandidates.push_back(L"points_selftest.csv");
    }

    wprintf(L"=== FrezaEgm: самотест ===\n");
    wprintf(L"Траектория: %s\n\n", trajectoryPath.c_str());

    // 1. Разбор файла траектории.
    const freza::TrajectoryLoadResult loaded = freza::LoadTrajectory(trajectoryPath);
    Check(loaded.ok, L"файл траектории прочитан");
    if (!loaded.ok)
    {
        wprintf(L"Ошибка: %s\n", loaded.error.c_str());
        return 1;
    }

    Check(loaded.points.size() == 5, L"в траектории 5 точек (замкнутый квадрат)");
    Check(CheckPoint(loaded.points[0], 750.0, 0.0, 600.0), L"точка 1 = (750, 0, 600)");
    Check(CheckPoint(loaded.points[1], 850.0, 0.0, 600.0), L"точка 2 = (850, 0, 600)");
    Check(CheckPoint(loaded.points[2], 850.0, 100.0, 600.0), L"точка 3 = (850, 100, 600)");
    Check(CheckPoint(loaded.points[3], 750.0, 100.0, 600.0), L"точка 4 = (750, 100, 600)");
    Check(CheckPoint(loaded.points[4], 750.0, 0.0, 600.0), L"точка 5 = (750, 0, 600) - квадрат замкнут");

    // 2. Сохранение и обратное чтение CSV.
    std::wstring csvPath;
    std::wstring error;
    bool saved = false;
    for (const std::wstring& candidate : csvCandidates)
    {
        error.clear();
        if (freza::WritePointsCsv(candidate, loaded.points, error))
        {
            csvPath = candidate;
            saved = true;
            break;
        }
    }

    Check(saved, L"CSV сохранён");
    if (saved)
    {
        wprintf(L"       файл: %s\n", csvPath.c_str());
    }
    else
    {
        wprintf(L"       ошибка записи: %s\n", error.c_str());
    }

    std::vector<freza::Point3> fromCsv;
    Check(saved && freza::ReadPointsCsv(csvPath, fromCsv, error), L"CSV прочитан обратно");
    Check(fromCsv.size() == loaded.points.size(), L"количество точек в CSV совпадает");
    if (fromCsv.size() == loaded.points.size())
    {
        bool same = true;
        for (size_t i = 0; i < fromCsv.size(); ++i)
        {
            if (!CheckPoint(fromCsv[i], loaded.points[i].x, loaded.points[i].y, loaded.points[i].z))
            {
                same = false;
            }
        }
        Check(same, L"координаты в CSV совпадают с исходными");
    }

    // 3. Перевод в смещения и интерполяция.
    const std::vector<freza::Point3> deltas = freza::ToDeltas(loaded.points);
    Check(deltas.size() == 5 && CheckPoint(deltas[0], 0.0, 0.0, 0.0), L"первое смещение = 0,0,0");
    Check(CheckPoint(deltas[2], 100.0, 100.0, 0.0), L"смещение третьей точки = 100,100,0");
    Check(CheckPoint(deltas[4], 0.0, 0.0, 0.0), L"последнее смещение = 0,0,0 (возврат домой)");

    std::vector<freza::Point3> dense;
    std::vector<int> dwells;
    freza::InterpolatePath(deltas, 2.0, 25.0, dense, dwells);
    Check(dense.size() > 100, L"интерполяция разбивает квадрат на мелкие шаги");
    Check(dense.size() == dwells.size(), L"таблица времени удержания совпадает по размеру");
    Check(CheckPoint(dense.front(), 0.0, 0.0, 0.0), L"интерполяция начинается в стартовой точке");
    Check(CheckPoint(dense.back(), 0.0, 0.0, 0.0), L"интерполяция заканчивается в стартовой точке");
    Check(dwells.size() > 1 && dwells[1] == 80, L"шаг 2 мм при подаче 25 мм/с = 80 мс");

    // 4. protobuf-сериализация.
    TestSerialization();

    // 5. UDP-обмен с заглушкой контроллера.
    TestUdpLoopback();

    wprintf(L"\n=== Итог: %s (ошибок: %d) ===\n", g_failures == 0 ? L"УСПЕХ" : L"ЕСТЬ ОШИБКИ", g_failures);
    ::WSACleanup();
    return g_failures == 0 ? 0 : 2;
}
