// trajectory.h — разбор файла траектории в массив точек.
//
// Поддерживаются два формата (определение автоматическое по содержимому):
//   * G-code (подмножество): G0/G1 с X/Y/Z, G90/G91, G20/G21, комментарии ; и (...).
//     Именно такой файл отдаёт любой CAM из STL, поэтому он основной.
//   * «простой XYZ»: по одной точке на строку, три числа (x y z), разделители —
//     пробелы, запятые или точки с запятой.
#pragma once

#include <string>
#include <vector>

namespace freza {

struct Point3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class TrajectoryFormat {
    Auto,     // определить по содержимому
    GCode,    // G-code (подмножество)
    XyzOnly,  // x y z построчно
};

struct TrajectoryLoadResult {
    bool ok = false;
    TrajectoryFormat detected = TrajectoryFormat::Auto;
    std::vector<Point3> points;   // абсолютные координаты, мм
    std::wstring error;           // заполняется, если ok == false
    std::wstring warnings;        // некритичные замечания парсера
};

// Читает файл и возвращает массив точек.
TrajectoryLoadResult LoadTrajectory(const std::wstring& path,
                                    TrajectoryFormat format = TrajectoryFormat::Auto);

// Определяет формат по первым непустым строкам текста.
TrajectoryFormat DetectFormat(const std::string& text);

// Переводит абсолютные точки в приращения относительно первой точки
// (первая точка становится 0,0,0). Так их ждёт EGM в режиме pose guidance.
std::vector<Point3> ToDeltas(const std::vector<Point3>& absolutePoints);

// Разбивает путь на мелкие шаги (линейная интерполяция) и считает время на шаг
// по заданной подаче. Нужно, чтобы робот шёл по сторонам, а не прыгал по углам.
// outIsCorner помечает точки, совпадающие с вершинами исходной траектории:
// на них контроллер должен реально доехать, иначе углы срезаются.
void InterpolatePath(const std::vector<Point3>& points,
                     double stepMm,
                     double feedMmPerSec,
                     std::vector<Point3>& outPoints,
                     std::vector<int>& outDwellMs,
                     std::vector<unsigned char>& outIsCorner);

void InterpolatePath(const std::vector<Point3>& points,
                     double stepMm,
                     double feedMmPerSec,
                     std::vector<Point3>& outPoints,
                     std::vector<int>& outDwellMs);

// "x=1.23 y=4.56 z=7.89" — для журнала/отладочных сообщений.
std::wstring FormatPoint(const Point3& point);

} // namespace freza
