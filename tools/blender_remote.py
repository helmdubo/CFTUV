"""Клиент remote-control моста Blender: JSON по TCP, `localhost:9876`.

Зачем он лежит в `tools/`, а не в одноразовом скрипте. Полевой цикл обязан
уметь задавать вопросы ЖИВОМУ сеансу Blender — тому, в котором владелец видит
сцену и держит несохранённые правки, — а не только фоновому процессу по файлу
на диске. Мост-аддон в сеансе владельца слушает порт 9876 и принимает
`{"type": ..., "params": {...}}`; отвечает `{"status": "success"|"error", ...}`.
Этот файл — единственное место, где протокол записан; батники и агенты зовут
его, а не заводят свои копии клиента.

Команды:

    python tools/blender_remote.py ping
        get_scene_info: жив ли мост и какая сцена открыта.

    python tools/blender_remote.py exec --code "import bpy; print(bpy.data.filepath)"
        Выполнить строку кода в живом сеансе.

    python tools/blender_remote.py run --file tools/run_envelope_mr1_building_gate.py \
        --timeout 1800 --script-args -- out.json all_seam_chains_l0
        Выполнить ФАЙЛ в живом сеансе как `__main__` (через runpy), с подменой
        `sys.argv`, как это делает сам Blender для `--python ... -- args`.

Таймаут по умолчанию 60 с: долгие полевые прогоны в живом сеансе блокируют
интерфейс владельца на всё время счёта, и это цена режима `-Live`, а не дефект
клиента. Для них поднимайте `--timeout` явно.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys


def send(payload: dict, port: int, timeout: float) -> dict:
    """Один запрос — один ответ. Ответ читается до первого полного JSON."""

    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as sock:
        sock.sendall(json.dumps(payload).encode("utf-8"))
        chunks: list[bytes] = []
        while True:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.append(chunk)
            try:
                return json.loads(b"".join(chunks).decode("utf-8"))
            except json.JSONDecodeError:
                continue
    raw = b"".join(chunks).decode("utf-8", "replace")
    raise SystemExit(
        "нет полного JSON-ответа от моста"
        + (f", получено: {raw[:2000]!r}" if raw else " (пустой ответ)")
    )


def run_file_code(path: str, script_args: list[str]) -> str:
    """Код, исполняющий файл в сеансе так же, как `blender --python file -- args`."""

    return (
        "import sys, runpy\n"
        f"_argv = [{path!r}] + {script_args!r}\n"
        "_backup = sys.argv[:]\n"
        "sys.argv = _argv\n"
        "try:\n"
        f"    runpy.run_path({path!r}, run_name='__main__')\n"
        "finally:\n"
        "    sys.argv = _backup\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("ping", "exec", "run"))
    parser.add_argument("--code", default="")
    parser.add_argument("--file", default="")
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--script-args",
        nargs=argparse.REMAINDER,
        default=[],
        help="всё после этого флага уходит в sys.argv исполняемого файла",
    )
    options = parser.parse_args()

    if options.command == "ping":
        payload = {"type": "get_scene_info"}
    elif options.command == "exec":
        if not options.code:
            raise SystemExit("exec требует --code")
        payload = {"type": "execute_code", "params": {"code": options.code}}
    else:
        if not options.file:
            raise SystemExit("run требует --file")
        payload = {
            "type": "execute_code",
            "params": {
                "code": run_file_code(options.file, list(options.script_args))
            },
        }

    reply = send(payload, options.port, options.timeout)
    print(json.dumps(reply, ensure_ascii=False))
    if reply.get("status") != "success":
        sys.exit(1)


if __name__ == "__main__":
    main()
