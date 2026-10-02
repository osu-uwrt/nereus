"""Freeze resolved scenario documents as session test fixtures.

The session tests replay recorded references (plant dynamics, mechanisms, prop world, whole sessions), so
they run on a frozen resolved document instead of resolving the live packs, which the team retunes. Absolute
asset paths are written as @NEREUS_SOURCE_DIR@/..., which CMake's configure_file fills in for each build.

Re-freeze only together with re-recording the references, from the source tree the references were recorded
with (a worktree of that commit works; assets must still exist at the same paths in the live tree):

    python tools/freeze_session_scenarios.py [--source DIR] talos_uwrt talos_uwrt_rpac
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "libraries" / "session" / "tests" / "fixtures"
PLACEHOLDER = "@NEREUS_SOURCE_DIR@/"


def freeze(source: Path, scenario: str) -> Path:
    """Resolve content/packs/scenarios/<scenario> of `source` and write the fixture; returns its path."""
    with tempfile.TemporaryDirectory() as folder:
        resolved = Path(folder) / "resolved.json"
        pack = source / "content" / "packs" / "scenarios" / scenario
        command = [sys.executable, "-m", "nereus.packs", "resolve", str(pack), "-o", str(resolved)]
        subprocess.run(command, check=True)
        text = resolved.read_text()
    if PLACEHOLDER.rstrip("/") in text:
        raise SystemExit(f"{scenario}: the document already contains {PLACEHOLDER}")
    text = text.replace(f"{source}/", PLACEHOLDER)
    if str(source) in text:
        raise SystemExit(f"{scenario}: a path into {source} is not under it with a separator")
    output = FIXTURES / f"{scenario}_resolved.json.in"
    output.write_text(json.dumps(json.loads(text), indent=1) + "\n")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--source", type=Path, default=ROOT, help="source tree to resolve (default: this one)"
    )
    parser.add_argument(
        "scenarios", nargs="+", help="scenario pack folder names under content/packs/scenarios"
    )
    args = parser.parse_args()
    for scenario in args.scenarios:
        print(freeze(args.source.resolve(), scenario).relative_to(ROOT))


if __name__ == "__main__":
    main()
