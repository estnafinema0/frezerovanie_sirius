// csv_io.h — сохранение и чтение массива точек в CSV.
//
// Формат (по требованию задачи): индекс точки и координаты в мм:
//     index,x,y,z
//     1,0.000000,0.000000,100.000000
//     ...
// Чтение терпимо к заголовку, разделителям ',' ';' и к отсутствию колонки индекса.
#pragma once

#include "trajectory.h"

#include <string>
#include <vector>

namespace freza {

bool WritePointsCsv(const std::wstring& path, const std::vector<Point3>& points, std::wstring& error);
bool ReadPointsCsv(const std::wstring& path, std::vector<Point3>& points, std::wstring& error);

} // namespace freza
