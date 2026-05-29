"""PlatformIO post-build hook: add .ino entries to compile_commands.json for clangd."""

import json
from pathlib import Path

Import("env")


def append_ino_entry(project_dir: Path, sketch_name: str) -> None:
    db_path = project_dir / "compile_commands.json"
    if not db_path.exists():
        return

    data = json.loads(db_path.read_text(encoding="utf-8"))
    ino_path = project_dir / sketch_name
    if any(str(ino_path) == item.get("file") for item in data):
        return

    entry = next(
        item
        for item in data
        if item["file"].replace("\\", "/").endswith("src/main.cpp")
    )
    command = entry["command"].replace("src\\main.cpp", sketch_name).replace(
        "src/main.cpp", sketch_name
    )
    data.append(
        {
            "directory": entry["directory"],
            "command": command,
            "file": str(ino_path),
            "output": entry["output"]
            .replace("src\\main.cpp", sketch_name)
            .replace("src/main.cpp", sketch_name),
        }
    )
    db_path.write_text(json.dumps(data, indent=4) + "\n", encoding="utf-8")


project_dir = Path(env["PROJECT_DIR"])
append_ino_entry(project_dir, f"{project_dir.name}.ino")
