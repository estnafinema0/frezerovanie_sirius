#include "csv_io.h"

#include "text_io.h"

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace freza {
namespace {

std::string Trim(const std::string& text)
{
    size_t begin = 0;
    size_t end = text.size();
    while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])) != 0)
    {
        ++begin;
    }
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0)
    {
        --end;
    }
    return text.substr(begin, end - begin);
}

std::vector<std::string> SplitFields(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    for (const char c : line)
    {
        if (c == ',' || c == ';' || c == '\t')
        {
            fields.push_back(Trim(current));
            current.clear();
        }
        else
        {
            current.push_back(c);
        }
    }
    fields.push_back(Trim(current));
    return fields;
}

bool ParseDouble(const std::string& token, double& value)
{
    if (token.empty())
    {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0' || errno == ERANGE)
    {
        return false;
    }
    value = parsed;
    return true;
}

} // namespace

bool WritePointsCsv(const std::wstring& path, const std::vector<Point3>& points, std::wstring& error)
{
    std::string text;
    text.reserve(points.size() * 48 + 32);
    text += "index,x,y,z\r\n";

    char line[192] = {};
    for (size_t i = 0; i < points.size(); ++i)
    {
        std::snprintf(line, sizeof(line), "%zu,%.6f,%.6f,%.6f\r\n",
                      i + 1, points[i].x, points[i].y, points[i].z);
        text += line;
    }

    return WriteFileBytes(path, text, error);
}

bool ReadPointsCsv(const std::wstring& path, std::vector<Point3>& points, std::wstring& error)
{
    std::string text;
    if (!ReadFileBytes(path, text, error))
    {
        return false;
    }

    std::vector<Point3> parsed;
    std::istringstream stream(text);
    std::string line;
    int badLines = 0;

    while (std::getline(stream, line))
    {
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        line = Trim(line);
        if (line.empty())
        {
            continue;
        }

        const std::vector<std::string> fields = SplitFields(line);
        std::vector<double> numbers;
        numbers.reserve(fields.size());
        for (const std::string& field : fields)
        {
            double value = 0.0;
            if (ParseDouble(field, value))
            {
                numbers.push_back(value);
            }
        }

        if (numbers.size() >= 4)
        {
            // index,x,y,z
            parsed.push_back(Point3{numbers[1], numbers[2], numbers[3]});
        }
        else if (numbers.size() == 3)
        {
            // x,y,z
            parsed.push_back(Point3{numbers[0], numbers[1], numbers[2]});
        }
        else
        {
            ++badLines; // заголовок или мусорная строка
        }
    }

    if (parsed.empty())
    {
        error = L"в CSV-файле не найдено ни одной точки";
        return false;
    }

    points = std::move(parsed);
    return true;
}

} // namespace freza
