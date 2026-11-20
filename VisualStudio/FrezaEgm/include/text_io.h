// text_io.h — чтение/запись файлов и конвертация UTF-8 <-> UTF-16.
#pragma once

#include <string>

namespace freza {

// Читает файл целиком в бинарном виде. Снимает BOM UTF-8, если он есть.
bool ReadFileBytes(const std::wstring& path, std::string& bytes, std::wstring& error);

// Пишет строку в файл в бинарном виде (перезаписывает).
bool WriteFileBytes(const std::wstring& path, const std::string& bytes, std::wstring& error);

std::wstring Utf8ToWide(const std::string& text);
std::string WideToUtf8(const std::wstring& text);

// Убирает BOM UTF-8 в начале строки (если есть).
void StripUtf8Bom(std::string& text);

} // namespace freza
