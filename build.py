#!/usr/bin/env python3
"""Builds the CDCL Foxglove converter extension and installs it into Foxglove Desktop.

Regenerates the converters from the current cdcl_umd_msgs first, so the
extension always matches the message definitions on disk.

    ./build.py                          generate, build, install
    ./build.py --msgs ~/other/cdcl_umd_msgs
    ./build.py --no-install             stop after producing the .foxe

Everything except the final copy runs in Docker, so no local Node is needed.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
IMAGE = "cdcl-foxglove-build"
EXT_ROOT = Path.home() / "snap" / "foxglove-studio" / "current" / ".foxglove-studio" / "extensions"

DEFAULT_MSG_PKG = "~/ros2_ws/src/cdcl_umd_msgs"

# Extension directories this build replaces, cleared on install.
SUPERSEDED = ("umd-cdcl.cdcl-converters-*", "umd-cdcl.cdcl-schema-converters-*",
              "umd-cdcl.cdcl-topic-converters-*")

# The message package is mounted read-only at /pkg, which the generator finds by
# default.
BUILD_STEPS = """
npm install --no-audit --no-fund --silent
python3 scripts/generate_converters.py /pkg
npm --workspace cdcl-converters run package
"""


def run(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(command, check=True, **kwargs)
    except FileNotFoundError:
        sys.exit(f"ERROR: {command[0]} not found. Docker is required.")
    except subprocess.CalledProcessError as error:
        sys.exit(f"ERROR: {command[0]} failed with exit code {error.returncode}")


def build(msg_pkg: Path) -> Path:
    run(["docker", "build", "-q", "-t", IMAGE, str(REPO_ROOT)], stdout=subprocess.DEVNULL)

    # Running as the host user keeps generated files owned by you.
    run([
        "docker", "run", "--rm",
        "-u", f"{os.getuid()}:{os.getgid()}",
        "-e", "HOME=/tmp",
        "-v", f"{REPO_ROOT}:/work",
        "-v", f"{msg_pkg}:/pkg:ro",
        "-w", "/work",
        IMAGE,
        "sh", "-euc", BUILD_STEPS,
    ])

    packages = sorted(
        (REPO_ROOT / "cdcl-converters").glob("*.foxe"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    if not packages:
        sys.exit("ERROR: no .foxe was produced")

    return packages[0]


def install(package: Path) -> None:
    if not EXT_ROOT.parent.is_dir():
        print("Foxglove Desktop (snap) not found; skipping install.")
        print(f"Upload {package} manually instead.")
        return

    manifest = json.loads((REPO_ROOT / "cdcl-converters" / "package.json").read_text())
    ext_id = f"{manifest['publisher']}.{manifest['name']}-{manifest['version']}"

    EXT_ROOT.mkdir(parents=True, exist_ok=True)

    for pattern in SUPERSEDED:
        for stale in EXT_ROOT.glob(pattern):
            shutil.rmtree(stale, ignore_errors=True)

    destination = EXT_ROOT / ext_id
    destination.mkdir(parents=True)

    source = REPO_ROOT / "cdcl-converters"
    shutil.copytree(source / "dist", destination / "dist")

    for name in ("package.json", "README.md", "CHANGELOG.md"):
        shutil.copy2(source / name, destination / name)

    print(f"Installed: {destination}")
    print()
    print("Fully quit and reopen Foxglove Studio to load the extension.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-m",
        "--msgs",
        default=os.environ.get("CDCL_MSG_PKG", DEFAULT_MSG_PKG),
        help=f"cdcl_umd_msgs package directory (default: {DEFAULT_MSG_PKG})",
    )
    parser.add_argument(
        "--no-install",
        action="store_true",
        help="Stop after producing the .foxe",
    )
    args = parser.parse_args()

    msg_pkg = Path(args.msgs).expanduser().resolve()

    if not (msg_pkg / "msg").is_dir():
        sys.exit(
            f"ERROR: no msg/ directory in {msg_pkg}\n"
            f"Pass --msgs with your cdcl_umd_msgs checkout."
        )

    print(f"Messages:  {msg_pkg}")

    package = build(msg_pkg)
    print(f"Packaged:  {package}")

    if not args.no_install:
        install(package)


if __name__ == "__main__":
    main()
