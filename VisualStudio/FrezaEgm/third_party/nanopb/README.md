# nanopb (вендорная копия)

Здесь лежат **только необходимые** файлы nanopb: рантайм и минимальный набор
генератора, достаточный, чтобы перегенерировать `egm3.pb.c/.h` без установки
nanopb в систему.

## Состав

| Путь | Что это |
|---|---|
| `pb.h`, `pb_common.c/.h`, `pb_encode.c/.h`, `pb_decode.c/.h` | рантайм: компилируется в цель `nanopb` |
| `generator/nanopb_generator.py` | генератор кода из `.proto` |
| `generator/protoc-gen-nanopb.bat` | обёртка-плагин для `protoc` (Windows) |
| `generator/proto/nanopb.proto`, `nanopb_pb2.py`, `_utils.py`, `__init__.py` | описание и Python-биндинги опций nanopb |

## Версия и лицензия

* Версия: **nanopb 1.0.0-dev** (ветка master, состояние 2024-12).
* Лицензия: **zlib** (см. `LICENSE.txt` в исходном дистрибутиве nanopb).
* Источник: <https://github.com/nanopb/nanopb> (копия разработчика:
  `C:\Users\Maksim\Downloads\nanopb-master\nanopb`).
* `generator/proto/nanopb_pb2.py` сгенерирован из `nanopb.proto` командой
  `protoc -I generator/proto --python_out=generator/proto generator/proto/nanopb.proto`
  (protoc 33.6, python-protobuf 7.34).

## Перегенерация кода EGM

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\generate_proto.ps1
```

Скрипт по умолчанию использует именно этот генератор. Требуется:
* `protoc` ≥ 3.15 (в проекте — 33.6 из `C:\Program Files\Protobuf\bin`);
* Python 3 с пакетом `protobuf` (`pip install protobuf`).

## Почему не полный protobuf

См. `docs/09-decisions.md`, ADR-001: MinGW-библиотеки `libprotobuf.a` из `Egm/lib`
несовместимы с MSVC, а nanopb — обычные C-файлы, которые MSVC компилирует
без дополнительных зависимостей.
