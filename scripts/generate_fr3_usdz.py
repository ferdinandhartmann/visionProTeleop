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

import numpy as np

from avp_stream.mujoco_msg.upload_xml import convert_and_download


REQUIRED_BODY_NAMES = (
    "fr3_link0",
    "fr3_link1",
    "fr3_link7",
    "robotiq_gripper_2f85",
    "left_outer_knuckle",
    "right_outer_knuckle",
    "ee_fk_xb",
    "ee_fk_yb",
    "ee_fk_zb",
    "ee_target_xb",
    "ee_target_yb",
    "ee_target_zb",
)


def convert_collada_to_obj(source: Path, destination: Path) -> None:
    """Bake a Franka COLLADA visual mesh into a MuJoCo-compatible OBJ."""
    namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(source).getroot()
    geometries = {
        geometry.attrib["id"]: geometry
        for geometry in root.findall(".//c:library_geometries/c:geometry", namespace)
    }
    output_vertices: list[np.ndarray] = []
    output_faces: list[tuple[int, int, int]] = []

    def append_geometry(geometry_id: str, transform: np.ndarray) -> None:
        geometry = geometries[geometry_id]
        mesh = geometry.find("c:mesh", namespace)
        if mesh is None:
            return
        sources = {}
        for source_element in mesh.findall("c:source", namespace):
            float_array = source_element.find("c:float_array", namespace)
            accessor = source_element.find("c:technique_common/c:accessor", namespace)
            if float_array is None or accessor is None:
                continue
            stride = int(accessor.attrib.get("stride", "1"))
            values = np.fromstring(float_array.text or "", sep=" ", dtype=np.float64)
            sources[source_element.attrib["id"]] = values.reshape(-1, stride)

        vertex_sources = {}
        for vertices in mesh.findall("c:vertices", namespace):
            position_input = next(
                (item for item in vertices.findall("c:input", namespace)
                 if item.attrib.get("semantic") == "POSITION"),
                None,
            )
            if position_input is not None:
                vertex_sources[vertices.attrib["id"]] = position_input.attrib["source"].lstrip("#")

        for triangles in mesh.findall("c:triangles", namespace):
            inputs = triangles.findall("c:input", namespace)
            vertex_input = next(
                item for item in inputs if item.attrib.get("semantic") == "VERTEX"
            )
            vertex_offset = int(vertex_input.attrib.get("offset", "0"))
            index_stride = max(int(item.attrib.get("offset", "0")) for item in inputs) + 1
            vertex_source_id = vertex_input.attrib["source"].lstrip("#")
            positions = sources[vertex_sources[vertex_source_id]][:, :3]

            homogeneous = np.column_stack((positions, np.ones(len(positions))))
            transformed = (transform @ homogeneous.T).T[:, :3]
            vertex_base = sum(len(batch) for batch in output_vertices)
            output_vertices.append(transformed)

            packed_indices = np.fromstring(
                triangles.findtext("c:p", default="", namespaces=namespace),
                sep=" ",
                dtype=np.int64,
            ).reshape(-1, index_stride)
            vertex_indices = packed_indices[:, vertex_offset].reshape(-1, 3)
            output_faces.extend(
                tuple(int(index) + vertex_base + 1 for index in face)
                for face in vertex_indices
            )

    def walk_node(node: ET.Element, parent_transform: np.ndarray) -> None:
        matrix_element = node.find("c:matrix", namespace)
        local = np.eye(4)
        if matrix_element is not None and matrix_element.text:
            local = np.fromstring(matrix_element.text, sep=" ").reshape(4, 4)
        world = parent_transform @ local
        for instance in node.findall("c:instance_geometry", namespace):
            append_geometry(instance.attrib["url"].lstrip("#"), world)
        for child in node.findall("c:node", namespace):
            walk_node(child, world)

    for scene in root.findall(".//c:library_visual_scenes/c:visual_scene", namespace):
        for node in scene.findall("c:node", namespace):
            walk_node(node, np.eye(4))

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as output:
        output.write(f"# Converted from {source.name}\n")
        for vertices in output_vertices:
            for x, y, z in vertices:
                output.write(f"v {x:.9g} {y:.9g} {z:.9g}\n")
        for a, b, c in output_faces:
            output.write(f"f {a} {b} {c}\n")


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

        # The runtime MJCF deliberately uses lightweight collision STLs. For
        # the packaged AR model, bake Franka's detailed DAE visual meshes to
        # OBJ because the deployed MuJoCo build has no COLLADA decoder.
        if relative.parts[-3:-1] == ("fr3", "collision"):
            visual_source = source.parent.parent / "visual" / f"{source.stem}.dae"
            if visual_source.is_file():
                destination = stage_dir / "assets/franka_visual" / f"{source.stem}.obj"
                convert_collada_to_obj(visual_source, destination)
                element.set("file", destination.relative_to(stage_dir).as_posix())
                continue
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
