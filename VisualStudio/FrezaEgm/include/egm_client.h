// egm_client.h — EGM-клиент: UDP-обмен с контроллером ABB + сериализация protobuf.
//
// Схема работы (режим pose guidance, как в RAPID EGMRunPose):
//   1. Клиент занимает UDP-порт (по умолчанию 6510) и ждёт первый пакет EgmRobot
//      от контроллера — это значит, что RAPID вошёл в сессию EGM.
//   2. На каждый пакет отвечает EgmSensor с полем planned.cartesian.pos,
//      где лежит СМЕЩЕНИЕ относительно позы в момент старта сессии (мм).
//   3. Каждая точка траектории удерживается dwellMs, затем отправляется следующая.
//   4. Ответы контроллера разбираются и складываются в снимок состояния + журнал.
//
// Протокол на проводе — protobuf 3 (схема egm3.proto), сериализация — nanopb.
#pragma once

#include "trajectory.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace freza {

struct EgmSettings {
    int port = 6510;              // UDP-порт, который слушает клиент (SIO.cfg: EGMSensor)
    int sendPeriodMs = 4;         // период отправки EgmSensor
    int dwellMs = 1000;           // пауза для одиночной ручной команды
    int waitRobotTimeoutMs = 120000; // сколько ждать первый пакет от контроллера
    double reachToleranceMm = 1.5;   // точка считается достигнутой, если TCP ближе
    int cornerSettleMs = 700;        // сколько максимум ждать на вершине траектории
    int maxStepMs = 3000;            // предохранитель для режима без интерполяции
};

enum class EgmState {
    Idle,          // не запущен
    WaitingRobot,  // порт занят, ждём пакет от контроллера
    Streaming,     // идёт отработка точек
    Finished,      // точки закончились, удерживаем последнюю
    Error,         // ошибка (текст в Snapshot::message)
};

const wchar_t* EgmStateText(EgmState state);

struct EgmSnapshot {
    EgmState state = EgmState::Idle;
    unsigned int seqno = 0;
    int pointIndex = 0;    // 1..pointCount, 0 — ещё не начали
    int pointCount = 0;
    Point3 target{};       // отправляемое смещение, мм
    Point3 feedback{};     // фактическая поза из EgmRobot, мм (если есть)
    bool hasFeedback = false;
    int mciState = 0;      // 0 undefined, 1 error, 2 stopped, 3 running
    int motorState = 0;    // 0 undefined, 1 on, 2 off
    int rapidState = 0;    // 0 undefined, 1 stopped, 2 running
    bool convergence = false;
    unsigned long long robotPackets = 0;
    unsigned long long sentPackets = 0;
    std::wstring message;  // человекочитаемое состояние/ошибка
};

class EgmClient {
public:
    EgmClient();
    ~EgmClient();

    EgmClient(const EgmClient&) = delete;
    EgmClient& operator=(const EgmClient&) = delete;

    // Запускает поток отработки. points — СМЕЩЕНИЯ (см. ToDeltas()).
    // При ошибке (занятый порт и т.п.) возвращает false и заполняет error.
    bool Start(const EgmSettings& settings, const std::vector<Point3>& deltas, std::wstring& error);

    // То же, но с индивидуальным временем удержания для каждой точки (интерполяция).
    bool Start(const EgmSettings& settings, const std::vector<Point3>& deltas,
               const std::vector<int>& dwellMsPerPoint, std::wstring& error);

    // То же плюс признаки «это вершина траектории»: на вершинах ждём достижения,
    // на прямых участках идём по времени (иначе темп задаёт только робот).
    bool Start(const EgmSettings& settings, const std::vector<Point3>& deltas,
               const std::vector<int>& dwellMsPerPoint,
               const std::vector<unsigned char>& cornerFlags, std::wstring& error);

    // Останавливает поток и освобождает порт.
    void Stop();

    bool Running() const;

    EgmSnapshot Snapshot() const;

    // Забирает новые строки журнала (с момента предыдущего вызова).
    std::vector<std::wstring> TakeLog();

    void Log(const std::wstring& line);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

// Собирает EgmSensor с декартовой коррекцией. Возвращает длину сообщения
// или 0 при ошибке сериализации.
size_t EncodeEgmSensor(const Point3& delta, uint32_t seqno, uint32_t tm,
                       unsigned char* buffer, size_t capacity);

// Собирает EgmRobot — используется в самотесте как «заглушка контроллера».
size_t EncodeEgmRobotTest(uint32_t seqno, unsigned char* buffer, size_t capacity);

} // namespace freza
