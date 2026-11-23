# Генерация C-кода nanopb из Egm/egm3.proto.
#
# Запуск из любого каталога:
#   pwsh -File tools\generate_proto.ps1
#
# Требуется:
#   * protoc >= 3.15 (в проекте проверено на 33.6 из C:\Program Files\Protobuf\bin);
#   * Python 3 + пакет protobuf (pip install protobuf).
# Генератор nanopb уже лежит в проекте: third_party\nanopb\generator.
#
# Результат кладётся в generated\egm3.pb.c и generated\egm3.pb.h.
# Эти файлы закоммичены в проект, поэтому для обычной сборки скрипт не нужен.
param(
    [string]$Protoc = "C:\Program Files\Protobuf\bin\protoc.exe",
    [string]$NanopbGenerator = (Join-Path $PSScriptRoot "..\third_party\nanopb\generator\protoc-gen-nanopb.bat"),
    [string]$ProtoDir = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\Egm")).Path,
    [string]$OutputDir = (Resolve-Path (Join-Path $PSScriptRoot "..\generated")).Path
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $Protoc)) { throw "Не найден protoc: $Protoc" }
if (-not (Test-Path $NanopbGenerator)) { throw "Не найден генератор nanopb: $NanopbGenerator" }
if (-not (Test-Path (Join-Path $ProtoDir "egm3.proto"))) { throw "Не найден egm3.proto в $ProtoDir" }
if (-not (Test-Path (Join-Path $ProtoDir "egm3.options"))) { throw "Не найден egm3.options в $ProtoDir" }

# Без этого комментарии на русском в сгенерированных файлах превращаются в «кракозябры».
$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

Push-Location $ProtoDir
try {
    # -I. обязателен: protoc запускается из каталога proto, чтобы nanopb нашёл
    # файл egm3.options (он ищется рядом с .proto).
    & $Protoc "--plugin=protoc-gen-nanopb=$NanopbGenerator" "--nanopb_out=$OutputDir" "-I." "egm3.proto"
    if ($LASTEXITCODE -ne 0) { throw "protoc завершился с кодом $LASTEXITCODE" }
}
finally {
    Pop-Location
}

Write-Host "Готово: $OutputDir\egm3.pb.c, $OutputDir\egm3.pb.h"
