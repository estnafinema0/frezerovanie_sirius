#include "trajectory.h"

#include "text_io.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <sstream>

namespace freza {
namespace {

constexpr double kEpsilon = 1e-9;

char UpperAscii(char c)
{
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

// Убирает комментарии двух видов: ";" до конца строки и "(...)" внутри строки.
std::string StripComments(const std::string& line)
{
    std::string result;
    result.reserve(line.size());
    int depth = 0;
    for (const char c : line)
    {
        if (c == '(')
        {
            ++depth;
            continue;
        }
        if (c == ')')
        {
            if (depth > 0)
            {
                --depth;
            }
            continue;
        }
        if (depth > 0)
        {
            continue;
        }
        if (c == ';')
        {
            break;
        }
        result.push_back(c);
    }
    return result;
}

std::vector<std::string> SplitWhitespace(const std::string& text)
{
    std::vector<std::string> tokens;
    std::string current;
    for (const char c : text)
    {
        if (std::isspace(static_cast<unsigned char>(c)) != 0)
        {
            if (!current.empty())
            {
                tokens.push_back(current);
                current.clear();
            }
        }
        else
        {
            current.push_back(c);
        }
    }
    if (!current.empty())
    {
        tokens.push_back(current);
    }
    return tokens;
}

bool ParseDouble(const char* begin, double& value)
{
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(begin, &end);
    if (end == begin || errno == ERANGE)
    {
        return false;
    }
    value = parsed;
    return true;
}

// Разбирает "X12.5" / "x-3" -> буква и число. Возвращает false, если нет числа.
bool ParseWord(const std::string& token, char& letter, double& value)
{
    if (token.empty())
    {
        return false;
    }
    letter = UpperAscii(token[0]);
    if (!std::isalpha(static_cast<unsigned char>(letter)))
    {
        return false;
    }
    return ParseDouble(token.c_str() + 1, value);
}

void PushPoint(std::vector<Point3>& points, const Point3& point)
{
    if (!points.empty())
    {
        const Point3& last = points.back();
        if (std::fabs(last.x - point.x) < kEpsilon &&
            std::fabs(last.y - point.y) < kEpsilon &&
            std::fabs(last.z - point.z) < kEpsilon)
        {
            return; // повтор точки не нужен
        }
    }
    points.push_back(point);
}

bool LooksLikeGCodeLine(const std::string& line)
{
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i])) != 0)
    {
        ++i;
    }
    if (i >= line.size())
    {
        return false;
    }
    const char first = UpperAscii(line[i]);
    if (std::strchr("GMTNS", first) == nullptr)
    {
        return false;
    }
    // После буквы должна идти цифра — иначе это не команда G-code.
    size_t j = i + 1;
    while (j < line.size() && std::isspace(static_cast<unsigned char>(line[j])) != 0)
    {
        ++j;
    }
    return j < line.size() && (std::isdigit(static_cast<unsigned char>(line[j])) != 0);
}

} // namespace

TrajectoryFormat DetectFormat(const std::string& text)
{
    std::istringstream stream(text);
    std::string line;
    int checked = 0;
    while (std::getline(stream, line) && checked < 200)
    {
        const std::string clean = StripComments(line);
        const std::vector<std::string> tokens = SplitWhitespace(clean);
        if (tokens.empty())
        {
            continue;
        }
        ++checked;
        if (LooksLikeGCodeLine(tokens.front()))
        {
            return TrajectoryFormat::GCode;
        }
        // Первое осмысленное слово — число, значит, это простой список XYZ.
        double dummy = 0.0;
        if (ParseDouble(tokens.front().c_str(), dummy))
        {
            return TrajectoryFormat::XyzOnly;
        }
    }
    return TrajectoryFormat::XyzOnly;
}

TrajectoryLoadResult LoadTrajectory(const std::wstring& path, TrajectoryFormat format)
{
    TrajectoryLoadResult result;
    std::string text;
    if (!ReadFileBytes(path, text, result.error))
    {
        return result;
    }

    result.detected = (format == TrajectoryFormat::Auto) ? DetectFormat(text) : format;

    std::vector<Point3> points;
    std::istringstream stream(text);
    std::string line;

    int skippedLines = 0;
    int arcLines = 0;

    // --- G-code -----------------------------------------------------------
    bool absolute = true;
    double scale = 1.0;       // мм на единицу файла (G20 -> 25.4)
    int motion = 1;           // модальная команда: 0 = G0, 1 = G1
    Point3 position{};        // текущая позиция

    while (std::getline(stream, line))
    {
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }

        const std::string clean = StripComments(line);
        const std::vector<std::string> tokens = SplitWhitespace(clean);
        if (tokens.empty())
        {
            continue;
        }

        if (result.detected == TrajectoryFormat::GCode)
        {
            bool hasCoordinate = false;
            Point3 next = position;

            for (const std::string& token : tokens)
            {
                char letter = 0;
                double value = 0.0;
                if (!ParseWord(token, letter, value))
                {
                    continue;
                }

                switch (letter)
                {
                case 'G':
                {
                    const int code = static_cast<int>(std::lround(value));
                    if (code == 0 || code == 1)
                    {
                        motion = code;
                    }
                    else if (code == 2 || code == 3)
                    {
                        ++arcLines; // дуги не поддерживаем: берём только конец
                        motion = code;
                    }
                    else if (code == 90)
                    {
                        absolute = true;
                    }
                    else if (code == 91)
                    {
                        absolute = false;
                    }
                    else if (code == 20)
                    {
                        scale = 25.4;
                    }
                    else if (code == 21)
                    {
                        scale = 1.0;
                    }
                    break;
                }
                case 'X':
                    next.x = absolute ? value * scale : next.x + value * scale;
                    hasCoordinate = true;
                    break;
                case 'Y':
                    next.y = absolute ? value * scale : next.y + value * scale;
                    hasCoordinate = true;
                    break;
                case 'Z':
                    next.z = absolute ? value * scale : next.z + value * scale;
                    hasCoordinate = true;
                    break;
                default:
                    break; // F, S, T, M, N и прочее для геометрии не важны
                }
            }

            if (hasCoordinate)
            {
                position = next;
                PushPoint(points, position);
            }
        }
        else
        {
            // --- простой XYZ ---------------------------------------------
            std::string normalized = clean;
            for (char& c : normalized)
            {
                if (c == ',' || c == ';')
                {
                    c = ' ';
                }
            }

            const std::vector<std::string> tokens2 = SplitWhitespace(normalized);
            std::vector<double> values;
            for (const std::string& token : tokens2)
            {
                double value = 0.0;
                if (ParseDouble(token.c_str(), value))
                {
                    values.push_back(value);
                }
            }

            if (values.size() >= 3)
            {
                PushPoint(points, Point3{values[0], values[1], values[2]});
            }
            else if (values.size() == 2)
            {
                PushPoint(points, Point3{values[0], values[1], 0.0});
            }
            else
            {
                ++skippedLines;
            }
        }
    }

    if (points.size() < 2)
    {
        result.error = L"в файле найдено меньше двух точек траектории";
        return result;
    }

    result.points = std::move(points);
    result.ok = true;

    if (result.detected == TrajectoryFormat::GCode)
    {
        if (arcLines > 0)
        {
            result.warnings += L"Дуги (G2/G3) не поддерживаются: для них взята только конечная точка. ";
        }
        if (!absolute)
        {
            result.warnings += L"Файл использует относительные координаты (G91). ";
        }
    }
    if (skippedLines > 0)
    {
        result.warnings += L"Пропущено строк без координат: " + std::to_wstring(skippedLines) + L". ";
    }

    return result;
}

std::vector<Point3> ToDeltas(const std::vector<Point3>& absolutePoints)
{
    std::vector<Point3> deltas;
    deltas.reserve(absolutePoints.size());
    if (absolutePoints.empty())
    {
        return deltas;
    }

    const Point3& origin = absolutePoints.front();
    for (const Point3& point : absolutePoints)
    {
        deltas.push_back(Point3{point.x - origin.x, point.y - origin.y, point.z - origin.z});
    }
    return deltas;
}

void InterpolatePath(const std::vector<Point3>& points,
                     double stepMm,
                     double feedMmPerSec,
                     std::vector<Point3>& outPoints,
                     std::vector<int>& outDwellMs)
{
    std::vector<unsigned char> corners;
    InterpolatePath(points, stepMm, feedMmPerSec, outPoints, outDwellMs, corners);
}

void InterpolatePath(const std::vector<Point3>& points,
                     double stepMm,
                     double feedMmPerSec,
                     std::vector<Point3>& outPoints,
                     std::vector<int>& outDwellMs,
                     std::vector<unsigned char>& outIsCorner)
{
    outPoints.clear();
    outDwellMs.clear();
    outIsCorner.clear();
    if (points.empty())
    {
        return;
    }

    const double step = (stepMm > 0.1) ? stepMm : 0.1;
    const double feed = (feedMmPerSec > 0.1) ? feedMmPerSec : 0.1;

    // Первая точка: короткая пауза, чтобы робот успел стабилизироваться.
    outPoints.push_back(points.front());
    outDwellMs.push_back(200);
    outIsCorner.push_back(1);

    for (size_t i = 1; i < points.size(); ++i)
    {
        const Point3& a = points[i - 1];
        const Point3& b = points[i];

        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (length < 1e-6)
        {
            continue;
        }

        const int steps = static_cast<int>(std::ceil(length / step));
        const double subLength = length / steps;
        const int dwell = static_cast<int>(std::lround(subLength / feed * 1000.0));

        for (int k = 1; k <= steps; ++k)
        {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            outPoints.push_back(Point3{a.x + dx * t, a.y + dy * t, a.z + dz * t});
            outDwellMs.push_back(std::max(1, dwell));
            // Вершина исходной траектории — только последняя подточка отрезка.
            outIsCorner.push_back((k == steps) ? 1 : 0);
        }
    }
}

std::wstring FormatPoint(const Point3& point)
{
    wchar_t buffer[128] = {};
    swprintf_s(buffer, L"x=%.3f y=%.3f z=%.3f", point.x, point.y, point.z);
    return std::wstring(buffer);
}

} // namespace freza
