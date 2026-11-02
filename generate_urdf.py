#!/usr/bin/env python3
"""
generate_urdf.py
----------------
Processes rover.urdf.xacro into a flat URDF that any visualizer can open,
without needing a running ROS environment.

It fakes ament_index_python.packages.get_package_share_directory() by:
  1. First checking the src/ tree (so edits are picked up live)
  2. Then falling back to the install/ tree for any built-only packages

Usage:
    python3 generate_urdf.py [output_file.urdf]

Output defaults to:  src/rover_description/urdf/rover.generated.urdf
"""

import sys
import os
import types

# ── 1. Locate workspace root ──────────────────────────────────────────────────
WORKSPACE = os.path.dirname(os.path.abspath(__file__))
INSTALL   = os.path.join(WORKSPACE, "install")
SRC       = os.path.join(WORKSPACE, "src")


# ── 2. Build a package-name → share-path map ─────────────────────────────────
# For source packages the "share" root is just the package source dir itself,
# because that is what $(find pkg) resolves to when run from the ROS overlay.
def _build_package_map():
    pkg_map = {}

    # (a) Source packages  →  src/<pkg>/  (preferred: picks up live edits)
    if os.path.isdir(SRC):
        for entry in os.listdir(SRC):
            pkg_dir = os.path.join(SRC, entry)
            if os.path.isfile(os.path.join(pkg_dir, "package.xml")):
                pkg_map[entry] = pkg_dir

    # (b) Install packages  →  install/<pkg>/share/<pkg>/  (fallback)
    if os.path.isdir(INSTALL):
        for entry in os.listdir(INSTALL):
            if entry in pkg_map:
                continue  # source wins
            share = os.path.join(INSTALL, entry, "share", entry)
            if os.path.isdir(share):
                pkg_map[entry] = share

    return pkg_map


PACKAGE_MAP = _build_package_map()


def _get_package_share_directory(package_name):
    if package_name in PACKAGE_MAP:
        return PACKAGE_MAP[package_name]
    raise RuntimeError(
        f"Package '{package_name}' not found.\n"
        f"Known packages: {sorted(PACKAGE_MAP.keys())}"
    )


# ── 3. Inject fake ament_index_python ────────────────────────────────────────
_pkg_mod = types.ModuleType("ament_index_python.packages")
_pkg_mod.get_package_share_directory = _get_package_share_directory

_top_mod = types.ModuleType("ament_index_python")
_top_mod.packages = _pkg_mod

sys.modules["ament_index_python"]          = _top_mod
sys.modules["ament_index_python.packages"] = _pkg_mod


# ── 4. Ensure xacro is importable ────────────────────────────────────────────
import importlib.util
if importlib.util.find_spec("xacro") is None:
    for candidate in [
        os.path.expanduser("~/.local/lib/python3.13/site-packages"),
        os.path.expanduser("~/.local/lib/python3.12/site-packages"),
    ]:
        if os.path.isdir(candidate):
            sys.path.insert(0, candidate)
            break

import xacro  # noqa: E402


# ── 5. Run xacro ─────────────────────────────────────────────────────────────
XACRO_FILE = os.path.join(SRC, "rover_description", "urdf", "rover.urdf.xacro")

output_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    SRC, "rover_description", "urdf", "rover.generated.urdf"
)

print(f"Processing : {XACRO_FILE}")
print(f"Output     : {output_path}")
print(f"Packages   : {sorted(PACKAGE_MAP.keys())}\n")

try:
    doc = xacro.process_file(XACRO_FILE, mappings={"use_sim_time": "false"})
    urdf_string = doc.toprettyxml(indent="  ")

    with open(output_path, "w") as f:
        f.write(urdf_string)

    print(f"\n  URDF written to:\n  {output_path}")
    print("\nOpen it with:")
    print("  - URDF Visualizer extension (morningfrog.urdf-visualizer)")
    print("  - Or drag into https://urdf.robotics.tools / https://gltf.report")

except Exception as e:
    print(f"\n  Error: {e}", file=sys.stderr)
    sys.exit(1)
