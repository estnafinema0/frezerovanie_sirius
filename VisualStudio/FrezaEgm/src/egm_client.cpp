#include "egm_client.h"

#include "egm3.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <algorithm>

namespace freza {
namespace {

std::wstring WithCode(const wchar_t* prefix, int code)
{
    wchar_t buffer[256] = {};
    swprintf_s(buffer, L"%s (код %d)", prefix, code);
    return std::wstring(buffer);
}

} // namespace

const wchar_t* EgmStateText(EgmState state)
{
    switch (state)
    {
    case EgmState::Idle:         return L"не запущен";
    case EgmState::WaitingRobot: return L"ожидание контроллера";
    case EgmState::Streaming:    return L"отработка траектории";
    case EgmState::Finished:     return L"траектория завершена (удержание)";
    case EgmState::Error:        return L"ошибка";
    }
    return L"?";
}

// ---------------------------------------------------------------------------
// Реализация
// ---------------------------------------------------------------------------

struct EgmClient::Impl {
    EgmSettings settings;
    std::vector<Point3> targets;
    std::vector<int> dwells;
    std::vector<unsigned char> corners;

    SOCKET sock = INVALID_SOCKET;
    std::thread worker;
    std::atomic<bool> stopRequested{false};
    std::atomic<bool> running{false};

    mutable std::mutex mutex;
    EgmSnapshot snapshot;
    std::vector<std::wstring> logs;

    void PushLog(const std::wstring& line)
    {
        std::lock_guard<std::mutex> lock(mutex);
        logs.push_back(line);
        if (logs.size() > 500)
        {
            logs.erase(logs.begin(), logs.begin() + static_cast<std::ptrdiff_t>(logs.size() - 500));
        }
    }

    void SetState(EgmState state, const std::wstring& message)
    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.state = state;
        snapshot.message = message;
    }

    void ApplyRobot(const EgmRobot& robot)
    {
        std::lock_guard<std::mutex> lock(mutex);
        ++snapshot.robotPackets;

        if (robot.has_mciState && robot.mciState.has_state)
        {
            snapshot.mciState = static_cast<int>(robot.mciState.state);
        }
        if (robot.has_motorState && robot.motorState.has_state)
        {
            snapshot.motorState = static_cast<int>(robot.motorState.state);
        }
        if (robot.has_rapidExecState && robot.rapidExecState.has_state)
        {
            snapshot.rapidState = static_cast<int>(robot.rapidExecState.state);
        }
        if (robot.has_mciConvergenceMet)
        {
            snapshot.convergence = robot.mciConvergenceMet;
        }
        if (robot.has_feedBack && robot.feedBack.has_cartesian && robot.feedBack.cartesian.has_pos)
        {
            const EgmCartesian& pos = robot.feedBack.cartesian.pos;
            snapshot.feedback = Point3{pos.x, pos.y, pos.z};
            snapshot.hasFeedback = true;
        }
    }

    void NoteSent(unsigned int seqno, int index, const Point3& target, unsigned long long total)
    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.seqno = seqno;
        snapshot.pointIndex = index + 1;
        snapshot.pointCount = static_cast<int>(targets.size());
        snapshot.target = target;
        snapshot.sentPackets = total;
    }

    // Доехал ли робот до точки (по фактической позе из EgmRobot.feedBack).
    bool ReachedTarget(const Point3& target, double tolerance)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!snapshot.hasFeedback)
        {
            return false;
        }
        const double dx = snapshot.feedback.x - target.x;
        const double dy = snapshot.feedback.y - target.y;
        const double dz = snapshot.feedback.z - target.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz) <= tolerance;
    }

    void Worker();
};

void EgmClient::Impl::Worker()
{
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::milliseconds;

    std::vector<unsigned char> rx(1600);
    std::vector<unsigned char> tx(512);

    unsigned int seqno = 0;
    unsigned long long sent = 0;
    unsigned long long received = 0;

    EgmState state = EgmState::WaitingRobot;
    int index = 0;

    sockaddr_in robotAddress{};
    int robotAddressLength = static_cast<int>(sizeof(robotAddress));
    bool haveRobotAddress = false;

    const Clock::time_point startedAt = Clock::now();
    Clock::time_point lastSend = startedAt;
    Clock::time_point pointStartedAt = startedAt;
    Clock::time_point lastRobotPacket = startedAt;

    SetState(EgmState::WaitingRobot, L"порт занят, ждём пакет от контроллера");

    while (!stopRequested.load())
    {
        robotAddressLength = static_cast<int>(sizeof(robotAddress));
        const int receivedBytes = ::recvfrom(sock,
                                             reinterpret_cast<char*>(rx.data()),
                                             static_cast<int>(rx.size()),
                                             0,
                                             reinterpret_cast<sockaddr*>(&robotAddress),
                                             &robotAddressLength);

        if (receivedBytes > 0)
        {
            EgmRobot robot = EgmRobot_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(rx.data(), static_cast<size_t>(receivedBytes));
            if (pb_decode(&stream, EgmRobot_fields, &robot))
            {
                ++received;
                haveRobotAddress = true;
                lastRobotPacket = Clock::now();
                ApplyRobot(robot);

                if (state == EgmState::WaitingRobot)
                {
                    state = EgmState::Streaming;
                    pointStartedAt = Clock::now();
                    PushLog(L"контроллер найден, начинаем поток EGM-сообщений");
                }

                // Не засоряем журнал: только первый и каждый 100-й пакет.
                if (received == 1 || (received % 100) == 0)
                {
                    wchar_t line[256] = {};
                    swprintf_s(line, L"EgmRobot: пакетов=%llu, seq=%u, MCI=%d, моторы=%d, сходимость=%s",
                               received,
                               robot.has_header && robot.header.has_seqno ? robot.header.seqno : 0u,
                               robot.has_mciState && robot.mciState.has_state ? static_cast<int>(robot.mciState.state) : 0,
                               robot.has_motorState && robot.motorState.has_state ? static_cast<int>(robot.motorState.state) : 0,
                               robot.has_mciConvergenceMet && robot.mciConvergenceMet ? L"да" : L"нет");
                    PushLog(line);
                }
            }
            else
            {
                PushLog(L"получен пакет, который не разобрался как EgmRobot");
            }
        }
        else if (receivedBytes == SOCKET_ERROR)
        {
            const int code = ::WSAGetLastError();
            if (code == WSAECONNRESET || code == WSAENETRESET)
            {
                // Контроллер закрыл EGM-сессию (программа дошла до EGMReset).
                PushLog(L"EGM-сессия закрыта контроллером — программа завершена");
                SetState(EgmState::Finished, L"сессия EGM закрыта контроллером");
                state = EgmState::Finished;
                break;
            }
            if (code != WSAETIMEDOUT && code != WSAEWOULDBLOCK)
            {
                PushLog(WithCode(L"ошибка приёма UDP", code));
                SetState(EgmState::Error, WithCode(L"ошибка приёма UDP", code));
                state = EgmState::Error;
                break;
            }
        }

        const Clock::time_point now = Clock::now();

        if (state == EgmState::WaitingRobot)
        {
            if (now - startedAt > Ms(settings.waitRobotTimeoutMs))
            {
                const std::wstring text = L"контроллер не вышел на связь за " +
                                          std::to_wstring(settings.waitRobotTimeoutMs) + L" мс";
                PushLog(text);
                SetState(EgmState::Error, text);
                state = EgmState::Error;
                break;
            }
            continue;
        }

        if (state == EgmState::Streaming)
        {
            const auto elapsed = now - pointStartedAt;
            const int dwell = dwells.empty() ? settings.dwellMs : dwells[static_cast<size_t>(index)];
            const bool isCorner = corners.empty() || corners[static_cast<size_t>(index)] != 0;

            bool advance = false;
            if (elapsed >= Ms(std::max(20, dwell)))
            {
                if (dwells.empty())
                {
                    // Без интерполяции — строго по времени удержания.
                    advance = true;
                }
                else if (!isCorner)
                {
                    // Прямой участок: цель движется по времени — движение плавное,
                    // без остановок на каждой подточке.
                    advance = true;
                }
                else
                {
                    // Вершина: короткая досылка, чтобы не срезать угол, но без
                    // длинных остановок.
                    advance = ReachedTarget(targets[static_cast<size_t>(index)], settings.reachToleranceMm) ||
                              elapsed >= Ms(settings.cornerSettleMs);
                }
            }

            if (advance)
            {
                if (index + 1 < static_cast<int>(targets.size()))
                {
                    ++index;
                    pointStartedAt = now;
                    PushLog(L"точка " + std::to_wstring(index + 1) + L" из " +
                            std::to_wstring(targets.size()) + L": " + FormatPoint(targets[static_cast<size_t>(index)]));
                }
                else
                {
                    state = EgmState::Finished;
                    PushLog(L"траектория завершена: удерживаем последнюю точку, нажмите «Стоп» для выхода");
                    SetState(EgmState::Finished, L"траектория завершена, удержание последней точки");
                }
            }
        }

        if (!haveRobotAddress)
        {
            continue;
        }

        if (now - lastSend < Ms(settings.sendPeriodMs))
        {
            continue;
        }

        const Point3& target = targets[static_cast<size_t>(index)];
        const size_t messageSize = EncodeEgmSensor(target,
                                                   seqno,
                                                   static_cast<uint32_t>(::GetTickCount()),
                                                   tx.data(),
                                                   tx.size());
        if (messageSize == 0)
        {
            PushLog(L"ошибка сериализации EgmSensor");
            SetState(EgmState::Error, L"ошибка сериализации EgmSensor");
            state = EgmState::Error;
            break;
        }

        const int sentBytes = ::sendto(sock,
                                       reinterpret_cast<const char*>(tx.data()),
                                       static_cast<int>(messageSize),
                                       0,
                                       reinterpret_cast<const sockaddr*>(&robotAddress),
                                       robotAddressLength);
        if (sentBytes == SOCKET_ERROR)
        {
            const int code = ::WSAGetLastError();
            if (code == WSAECONNRESET || code == WSAENETRESET)
            {
                PushLog(L"EGM-сессия закрыта контроллером — программа завершена");
                SetState(EgmState::Finished, L"сессия EGM закрыта контроллером");
                state = EgmState::Finished;
                break;
            }
            PushLog(WithCode(L"ошибка отправки UDP", code));
        }
        else
        {
            ++sent;
            NoteSent(seqno, index, target, sent);
            ++seqno;
            lastSend = now;
        }

        if (now - lastRobotPacket > std::chrono::seconds(2) && !stopRequested.load())
        {
            // Контроллер молчит: EGM-сессия, скорее всего, закрыта.
            SetState(state, L"нет пакетов от контроллера более 2 с");
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot.robotPackets = received;
        snapshot.sentPackets = sent;
        if (snapshot.state != EgmState::Error)
        {
            snapshot.state = EgmState::Idle;
            snapshot.message = L"остановлено";
        }
    }

    running = false;
}

// ---------------------------------------------------------------------------
// Публичный интерфейс
// ---------------------------------------------------------------------------

EgmClient::EgmClient() : m_impl(new Impl())
{
    WSADATA data{};
    ::WSAStartup(MAKEWORD(2, 2), &data);
}

EgmClient::~EgmClient()
{
    Stop();
    if (m_impl->sock != INVALID_SOCKET)
    {
        ::closesocket(m_impl->sock);
        m_impl->sock = INVALID_SOCKET;
    }
    ::WSACleanup();
}

bool EgmClient::Start(const EgmSettings& settings, const std::vector<Point3>& deltas, std::wstring& error)
{
    return Start(settings, deltas, std::vector<int>(), error);
}

bool EgmClient::Start(const EgmSettings& settings, const std::vector<Point3>& deltas,
                      const std::vector<int>& dwellMsPerPoint, std::wstring& error)
{
    return Start(settings, deltas, dwellMsPerPoint, std::vector<unsigned char>(), error);
}

bool EgmClient::Start(const EgmSettings& settings, const std::vector<Point3>& deltas,
                      const std::vector<int>& dwellMsPerPoint,
                      const std::vector<unsigned char>& cornerFlags, std::wstring& error)
{
    Stop();

    if (!dwellMsPerPoint.empty() && dwellMsPerPoint.size() != deltas.size())
    {
        error = L"некорректная таблица времени удержания точек";
        return false;
    }
    if (!cornerFlags.empty() && cornerFlags.size() != deltas.size())
    {
        error = L"некорректная таблица вершин траектории";
        return false;
    }

    if (deltas.empty())
    {
        error = L"список точек пуст";
        return false;
    }
    if (settings.port <= 0 || settings.port > 65535)
    {
        error = L"некорректный UDP-порт";
        return false;
    }
    if (settings.sendPeriodMs < 1)
    {
        error = L"период отправки должен быть не меньше 1 мс";
        return false;
    }

    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.targets = deltas;
    impl.dwells = dwellMsPerPoint;
    impl.corners = cornerFlags;

    SOCKET sock = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        error = WithCode(L"не удалось создать UDP-сокет", ::WSAGetLastError());
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<u_short>(settings.port));

    if (::bind(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR)
    {
        const int code = ::WSAGetLastError();
        ::closesocket(sock);
        error = WithCode(L"не удалось занять UDP-порт (возможно, он уже используется)", code);
        return false;
    }

    const DWORD timeoutMs = 200;
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));

    impl.sock = sock;
    impl.stopRequested = false;
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        impl.snapshot = EgmSnapshot{};
        impl.snapshot.pointCount = static_cast<int>(deltas.size());
        impl.snapshot.state = EgmState::WaitingRobot;
        impl.snapshot.message = L"запуск";
    }
    {
        std::lock_guard<std::mutex> lock(impl.mutex);
        impl.logs.clear();
    }

    impl.running = true;
    impl.worker = std::thread([&impl]() { impl.Worker(); });
    return true;
}

void EgmClient::Stop()
{
    Impl& impl = *m_impl;
    impl.stopRequested = true;

    if (impl.worker.joinable())
    {
        impl.worker.join();
    }

    if (impl.sock != INVALID_SOCKET)
    {
        ::closesocket(impl.sock);
        impl.sock = INVALID_SOCKET;
    }

    impl.running = false;

    std::lock_guard<std::mutex> lock(impl.mutex);
    if (impl.snapshot.state != EgmState::Error)
    {
        impl.snapshot.state = EgmState::Idle;
        impl.snapshot.pointIndex = 0;
    }
}

bool EgmClient::Running() const
{
    return m_impl->running.load();
}

EgmSnapshot EgmClient::Snapshot() const
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->snapshot;
}

std::vector<std::wstring> EgmClient::TakeLog()
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    std::vector<std::wstring> lines;
    lines.swap(m_impl->logs);
    return lines;
}

void EgmClient::Log(const std::wstring& line)
{
    m_impl->PushLog(line);
}

// ---------------------------------------------------------------------------
// Сериализация
// ---------------------------------------------------------------------------

size_t EncodeEgmSensor(const Point3& delta, uint32_t seqno, uint32_t tm,
                       unsigned char* buffer, size_t capacity)
{
    EgmSensor message = EgmSensor_init_zero;

    message.has_header = true;
    message.header.has_seqno = true;
    message.header.seqno = seqno;
    message.header.has_tm = true;
    message.header.tm = tm;
    message.header.mtype = EgmHeader_MessageType_MSGTYPE_CORRECTION;

    message.has_planned = true;
    message.planned.has_cartesian = true;
    message.planned.cartesian.has_pos = true;

    // Нули отправляем явно (has_*), иначе proto3 их выбросит и контроллер,
    // который разбирает сообщение как proto2, не увидит required-полей.
    message.planned.cartesian.pos.has_x = true;
    message.planned.cartesian.pos.x = delta.x;
    message.planned.cartesian.pos.has_y = true;
    message.planned.cartesian.pos.y = delta.y;
    message.planned.cartesian.pos.has_z = true;
    message.planned.cartesian.pos.z = delta.z;

    // Ориентация: единичный кватернион (без поворота). Без orient/euler контроллер
    // может считать позу неполной и не применять коррекцию положения.
    message.planned.cartesian.has_orient = true;
    message.planned.cartesian.orient.has_u0 = true;
    message.planned.cartesian.orient.u0 = 1.0;
    message.planned.cartesian.orient.has_u1 = true;
    message.planned.cartesian.orient.u1 = 0.0;
    message.planned.cartesian.orient.has_u2 = true;
    message.planned.cartesian.orient.u2 = 0.0;
    message.planned.cartesian.orient.has_u3 = true;
    message.planned.cartesian.orient.u3 = 0.0;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, capacity);
    if (!pb_encode(&stream, EgmSensor_fields, &message))
    {
        return 0;
    }
    return stream.bytes_written;
}

size_t EncodeEgmRobotTest(uint32_t seqno, unsigned char* buffer, size_t capacity)
{
    EgmRobot message = EgmRobot_init_zero;

    message.has_header = true;
    message.header.has_seqno = true;
    message.header.seqno = seqno;
    message.header.has_tm = true;
    message.header.tm = static_cast<uint32_t>(::GetTickCount());
    message.header.mtype = EgmHeader_MessageType_MSGTYPE_DATA;

    message.has_mciState = true;
    message.mciState.has_state = true;
    message.mciState.state = EgmMCIState_MCIStateType_MCI_RUNNING;

    message.has_motorState = true;
    message.motorState.has_state = true;
    message.motorState.state = EgmMotorState_MotorStateType_MOTORS_ON;

    message.has_mciConvergenceMet = true;
    message.mciConvergenceMet = true;

    message.has_feedBack = true;
    message.feedBack.has_cartesian = true;
    message.feedBack.cartesian.has_pos = true;
    message.feedBack.cartesian.pos.has_x = true;
    message.feedBack.cartesian.pos.x = 750.0;
    message.feedBack.cartesian.pos.has_y = true;
    message.feedBack.cartesian.pos.y = 0.0;
    message.feedBack.cartesian.pos.has_z = true;
    message.feedBack.cartesian.pos.z = 1123.0;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, capacity);
    if (!pb_encode(&stream, EgmRobot_fields, &message))
    {
        return 0;
    }
    return stream.bytes_written;
}

} // namespace freza
