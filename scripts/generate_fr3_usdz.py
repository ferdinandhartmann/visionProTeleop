#!/usr/bin/env python3
"""Build and validate the packaged FR3/Robotiq USDZ asset.

This is a developer/release tool, not a launch-time dependency. It creates an
isolated conversion bundle containing only the MJCF and its referenced meshes,
so converter uploads can never walk a shared directory such as /tmp.
"""

from __future__ import annotations

import argparse
import shutil
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from avp_stream.mujoco_msg.upload_xml import convert_and_download


REQUIRED_BODY_NAMES = (
    "fr3_link0",
    "fr3_link1",
    "fr3_link7",
    "robotiq_gripper_2f85",
    "left_outer_knuckle",
    "right_outer_knuckle",
)


def stage_scene(xml_path: Path, description_root: Path, stage_dir: Path) -> Path:
    """Copy the MJCF and exactly its referenced Franka assets into stage_dir."""
    xml_text = xml_path.read_text(encoding="utf-8")
    root = ET.fromstring(xml_text)
    staged_prefix = "assets/franka_description"

    for element in root.iter():
        file_value = element.attrib.get("file")
        if not file_value or "${franka_description}" not in file_value:
            continue
        relative = Path(file_value.replace("${franka_description}/", ""))
        source = description_root / relative
        if not source.is_file():
            raise FileNotFoundError(f"Referenced mesh does not exist: {source}")
        destination = stage_dir / staged_prefix / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        element.set("file", f"{staged_prefix}/{relative.as_posix()}")

    staged_xml = stage_dir / xml_path.name
    ET.ElementTree(root).write(staged_xml, encoding="unicode")
    return staged_xml


def validate_usdz(path: Path) -> None:
    """Check packaging and the body names required by live pose updates."""
    if not zipfile.is_zipfile(path):
        raise ValueError(f"Not a valid USDZ/ZIP package: {path}")
    with zipfile.ZipFile(path) as archive:
        payload = b"".join(archive.read(name) for name in archive.namelist())
    missing = [name for name in REQUIRED_BODY_NAMES if name.encode() not in payload]
    if missing:
        raise ValueError(f"USDZ is missing required body names: {missing}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--franka-description", type=Path, required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("ros2_ws/src/robot_description/franka_mujoco/fr3_robotiq_2f85.usdz"),
    )
    parser.add_argument("--server", default="http://mujoco-usd-convert.xyz")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="fr3_usdz_bundle_") as temp:
        stage_dir = Path(temp)
        staged_xml = stage_scene(args.xml.resolve(), args.franka_description.resolve(), stage_dir)
        converted = convert_and_download(args.server.rstrip("/"), staged_xml, stage_dir / "output")
        validate_usdz(converted)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(converted, args.output)

    print(f"Validated FR3 USDZ: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
