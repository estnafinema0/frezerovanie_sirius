#include "text_io.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <fstream>

namespace freza {

bool ReadFileBytes(const std::wstring& path, std::string& bytes, std::wstring& error)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file)
    {
        error = L"не удалось открыть файл: " + path;
        return false;
    }

    const std::streamoff size = file.tellg();
    if (size < 0)
    {
        error = L"не удалось определить размер файла: " + path;
        return false;
    }

    bytes.resize(static_cast<size_t>(size));
    file.seekg(0, std::ios::beg);
    if (size > 0)
    {
        file.read(&bytes[0], size);
        if (!file)
        {
            error = L"ошибка чтения файла: " + path;
            return false;
        }
    }

    StripUtf8Bom(bytes);
    return true;
}

bool WriteFileBytes(const std::wstring& path, const std::string& bytes, std::wstring& error)
{
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file)
    {
        error = L"не удалось создать файл: " + path;
        return false;
    }

    if (!bytes.empty())
    {
        file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

    if (!file)
    {
        error = L"ошибка записи файла: " + path;
        return false;
    }

    return true;
}

std::wstring Utf8ToWide(const std::string& text)
{
    if (text.empty())
    {
        return std::wstring();
    }

    const int size = ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
    if (size <= 0)
    {
        return std::wstring();
    }

    std::wstring result(static_cast<size_t>(size), L'\0');
    ::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), &result[0], size);
    return result;
}

std::string WideToUtf8(const std::wstring& text)
{
    if (text.empty())
    {
        return std::string();
    }

    const int size = ::WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
                                           nullptr, 0, nullptr, nullptr);
    if (size <= 0)
    {
        return std::string();
    }

    std::string result(static_cast<size_t>(size), '\0');
    ::WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()),
                          &result[0], size, nullptr, nullptr);
    return result;
}

void StripUtf8Bom(std::string& text)
{
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF)
    {
        text.erase(0, 3);
    }
}

} // namespace freza
