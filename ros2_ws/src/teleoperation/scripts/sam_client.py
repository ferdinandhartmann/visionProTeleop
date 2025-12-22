#!/usr/bin/env python3

from urllib import response
import rclpy
from rclpy.node import Node

from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from pathlib import Path
import requests
import time
import pyheif
from PIL import Image
import io
import json
import ament_index_python.packages

import xml.etree.ElementTree as ET
import random

class SamClient(Node):

    def __init__(self):
        super().__init__("sam_client")
        self.get_logger().info("SAM client started")

        self.pending = {}  # basename -> files
        
        self.prompt_name = None

        # Load parameters
        self.declare_parameter("watch_dir", str(Path.home() / "Dropbox/sam_data"))
        self.declare_parameter("server_url", "http://localhost:18000")
        self.declare_parameter("result_files", ["model1.obj", "model2.obj", "material.mtl", "texture.png"])

        self.watch_dir = Path(self.get_parameter("watch_dir").value)
        self.server_url = self.get_parameter("server_url").value
        self.result_files = self.get_parameter("result_files").value
        
        # Additional parameters for MuJoCo injection
        self.declare_parameter("add_to_mujoco", False)
        self.declare_parameter("mujoco_scene_name", "scene_mycobot")
        
        # Find mujoco_scene.xml in robot_description package if not absolute
        mujoco_scene_name = self.get_parameter("mujoco_scene_name").value
        robot_description_dir = ament_index_python.packages.get_package_share_directory("robot_description")
        self.mujoco_scene_path = Path(robot_description_dir) / "mycobot_mujoco" / f"{mujoco_scene_name}.xml"
        
        self.add_to_mujoco = self.get_parameter("add_to_mujoco").value


        self.check_health()

        event_handler = FolderHandler(self)
        observer = Observer()
        observer.schedule(event_handler, str(self.watch_dir), recursive=False)
        observer.start()
        self.get_logger().info(f"Watching directory: {self.watch_dir}")

    def check_health(self):
        self.get_logger().info("Checking server health...")
        while True:
            try:
                response = requests.get(f"{self.server_url}/health", verify=False)
                if response.status_code == 200:
                    self.get_logger().info("Server health check passed.")
                    break
                else:
                    self.get_logger().error(f"Server health check failed with status code: {response.status_code}")
            except requests.RequestException as e:
                self.get_logger().error(f"Server health check failed: {e}")
            self.get_logger().info("Retrying server health check in 2 seconds...")
            time.sleep(2)

        self.get_logger().info("Checking if server workers are ready...")
        while True:
            try:
                r = requests.get(f"{self.server_url}/ready", verify=False)
                data = r.json()
                if data.get("ready"):
                    self.get_logger().info("Server workers are ready!")
                    break
                else:
                    self.get_logger().info("Server workers not ready yet, retrying ...")
                    time.sleep(2)
            except requests.RequestException as e:
                self.get_logger().error(f"Error checking server readiness: {e}")
                time.sleep(2)
            except json.JSONDecodeError:
                self.get_logger().error("Invalid JSON from server /ready, retrying...")
                time.sleep(2)

    def try_process(self, basename):
        files = self.pending.get(basename, {})
        self.get_logger().info(f"Trying to process: {basename}, files: {list(files.keys())}")
        if "img" in files and "txt" in files:
            self.get_logger().info(f"Found complete pair: {basename}")
            self.process_job(files["img"], files["txt"])
            del self.pending[basename]


    def process_job(self, heic_path, txt_path):
        self.get_logger().info(f"Processing job for: {heic_path}, {txt_path}")
        if heic_path.suffix.lower() == ".heic":
            jpg_path = heic_path.with_suffix(".jpg")
            self.get_logger().info(f"Converting {heic_path} to {jpg_path}")
            heif_file = pyheif.read(heic_path)
            image = Image.frombytes(
                heif_file.mode,
                heif_file.size,
                heif_file.data
            )
            image.save(jpg_path, "JPEG")
            self.get_logger().info(f"Saved JPG: {jpg_path}")
        elif heic_path.suffix.lower() in [".jpg", ".jpeg"]:
            jpg_path = heic_path
            self.get_logger().info(f"Image is already JPEG: {jpg_path}")
        else:
            self.get_logger().error(f"Unsupported image format: {heic_path.suffix}")
            return

        prompt = txt_path.read_text().strip()
        self.get_logger().info(f"Read prompt: {prompt}")

        self.get_logger().info(f"Sending image and prompt to server: {self.server_url}")
        with open(jpg_path, "rb") as img:
            response = requests.post(
                f"{self.server_url}/submit",
                files={"image": img},
                data={"prompt": prompt},
                verify=False
            )

        try:
            job_id = response.json()["job_id"]
        except json.JSONDecodeError:
            self.get_logger().error(f"Server returned invalid response: {response.text}")
            return
        self.get_logger().info(f"Job submitted: {job_id}")

        self.poll_result(job_id)

    def poll_result(self, job_id):
        self.get_logger().info(f"Polling for result of job: {job_id}")
        while True:
            r = requests.get(f"{self.server_url}/status/{job_id}", verify=False)
            try:
                data = r.json()
            except json.JSONDecodeError:
                self.get_logger().error("Server returned invalid JSON, retrying...")
                time.sleep(4)
                continue
            self.get_logger().info(f"Job status: {data['status']}")
            if data["status"] == "done":
                self.get_logger().info(f"Job {job_id} done, downloading results")
                self.download_results(job_id)
                break
            time.sleep(4)

    def download_results(self, job_id):
        # Install folder (current)
        robot_description_dir = ament_index_python.packages.get_package_share_directory("robot_description")
        install_out_dir = Path(robot_description_dir) / "mycobot_mujoco" / "meshes_mujoco" / "sam_models"
        install_out_dir.mkdir(parents=True, exist_ok=True)
        
        # Source folder (workspace src)
        workspace_src_dir = Path.home() / "visionpro_teleop_project/visionProTeleop/ros2_ws/src/robot_description/mycobot_mujoco/meshes_mujoco/sam_models"
        workspace_src_dir.mkdir(parents=True, exist_ok=True)

        # First, get list of files for this job
        r = requests.get(f"{self.server_url}/list/{job_id}", verify=False)
        try:
            files = r.json().get("files", [])
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to get file list for job {job_id}")
            return

        if not files:
            self.get_logger().info(f"No files found for job {job_id}")
            return

        self.get_logger().info(f"Downloading files: {files}")

        for fname in files:
            self.get_logger().info(f"Downloading {fname}")
            r = requests.get(f"{self.server_url}/download/{job_id}/{fname}", verify=False)
            # Save to both folders
            (install_out_dir / fname).write_bytes(r.content)
            (workspace_src_dir / fname).write_bytes(r.content)
            self.get_logger().info(f"Saved {fname} to {install_out_dir}\nand {workspace_src_dir}")
            
            # If the file is a PNG and ends with '_material.png', save its name
            if fname.endswith("_material.png"):
                self.prompt_name = fname[:-len("_material.png")]
                self.get_logger().info(f"Prompt/Object name identified: {self.prompt_name}")

        self.get_logger().info("All results downloaded")
        
        if self.add_to_mujoco:
            self.get_logger().info("add_to_mujoco enabled -> updating MuJoCo scene")
            self.add_object_to_mujoco(files)



    def add_object_to_mujoco(self, files):
        """
        Injects a SAM-generated object into the MuJoCo scene XML.
        Assumes files contain:
        - *_visual.obj
        - *_collision.obj
        - *.png (texture)
        """

        if not self.mujoco_scene_path.exists():
            self.get_logger().error(f"MuJoCo scene not found: {self.mujoco_scene_path}")
            return
        self.get_logger().info(f"Modifying MuJoCo scene: {self.mujoco_scene_path}")

        tree = ET.parse(self.mujoco_scene_path)
        root = tree.getroot()

        asset = root.find("asset")
        worldbody = root.find("worldbody")

        if asset is None or worldbody is None:
            self.get_logger().error("Invalid MuJoCo XML: missing <asset> or <worldbody>")
            return
        
        # --- infer names ---
        # Get all files that start with the object name (self.prompt_name)
        obj_files = [f for f in files if f.startswith(self.prompt_name)]
        texture_file = next(f for f in obj_files if f.endswith(".png"))
        visual_mesh = next(f for f in obj_files if "visual" in f and f.endswith(".obj"))
        collision_mesh = next(f for f in obj_files if "collision" in f and f.endswith(".obj"))

        # Prevent duplicates
        if worldbody.find(f".//body[@name='{self.prompt_name}']") is not None:
            self.get_logger().info(f"Object '{self.prompt_name}' already exists in scene, skipping")
            return

        # ---------- ASSET ----------
        ET.SubElement(
            asset,
            "texture",
            name=f"{self.prompt_name}_tex",
            type="2d",
            file=f"meshes_mujoco/sam_models/{self.prompt_name}_material.png"
        )

        ET.SubElement(
            asset,
            "material",
            name=f"{self.prompt_name}_mat",
            texture=f"{self.prompt_name}_tex",
            texrepeat="2 2",
            reflectance="0",
            shininess="0",
            specular="0"
        )

        ET.SubElement(
            asset,
            "mesh",
            name=f"{self.prompt_name}_visual",
            file=f"sam_models/{visual_mesh}"
        )

        ET.SubElement(
            asset,
            "mesh",
            name=f"{self.prompt_name}_collision",
            file=f"sam_models/{collision_mesh}"
        )

        # ---------- WORLDBODY ----------
        body = ET.SubElement(
            worldbody,
            "body",
            name=self.prompt_name,
            pos=f"{random.uniform(-0.4, 0.4):.2f} {random.uniform(-0.4, 0.4):.2f} {random.uniform(0.3, 0.8):.2f}",
            euler="0.0 0.3 0"
        )

        ET.SubElement(body, "joint", type="free")

        ET.SubElement(
            body,
            "geom",
            type="mesh",
            mesh=f"{self.prompt_name}_visual",
            material=f"{self.prompt_name}_mat",
            contype="0",
            conaffinity="0"
        )

        ET.SubElement(
            body,
            "geom",
            type="mesh",
            mesh=f"{self.prompt_name}_collision",
            mass="0.1",
            contype="1",
            conaffinity="1"
        )


        ET.indent(tree, space="  ", level=0)
        
        tree.write(self.mujoco_scene_path, encoding="utf-8", xml_declaration=True)
        
        
        self.extend_home_keyframe_via_include()

        self.get_logger().info(f"Added object '{self.prompt_name}' into MuJoCo scene.\nFINISHED!")
            
            
    def extend_home_keyframe_via_include(self):
        """
        Finds the <include file="..."> in the scene XML,
        opens the included robot XML,
        and appends 7 zeros to the 'home' keyframe qpos
        (idempotent).
        """

        # --- Parse scene XML ---
        scene_tree = ET.parse(self.mujoco_scene_path)
        scene_root = scene_tree.getroot()

        include = scene_root.find("include")
        if include is None or "file" not in include.attrib:
            self.get_logger().error("No <include file='...'> found in MuJoCo scene")
            return

        include_file = include.attrib["file"]

        # Resolve path relative to scene file
        robot_xml_path = (self.mujoco_scene_path.parent / include_file).resolve()

        if not robot_xml_path.exists():
            self.get_logger().error(f"Included MuJoCo file not found: {robot_xml_path}")
            return

        self.get_logger().info(f"Modifying home keyframe in: {robot_xml_path}")

        # --- Parse robot XML ---
        robot_tree = ET.parse(robot_xml_path)
        robot_root = robot_tree.getroot()

        keyframe = robot_root.find("keyframe")
        if keyframe is None:
            self.get_logger().error("No <keyframe> block found in robot XML")
            return

        home = keyframe.find("key[@name='home']")
        if home is None or "qpos" not in home.attrib:
            self.get_logger().error("No 'home' keyframe with qpos found")
            return

        # --- Modify qpos ---
        qpos_values = home.attrib["qpos"].split()
        original_len = len(qpos_values)

        qpos_values.extend(["0"] * 7)
        home.attrib["qpos"] = " ".join(qpos_values)
        
        ET.indent(robot_tree, space="  ", level=0)
        robot_tree.write(robot_xml_path, encoding="utf-8", xml_declaration=True)

        self.get_logger().info(
            f"Extended home qpos from {original_len} to {len(qpos_values)} entries"
        )



class FolderHandler(FileSystemEventHandler):

    def __init__(self, node):
        self.node = node

    def on_created(self, event):
        if event.is_directory:
            return
        path = Path(event.src_path)
        base = path.stem

        # Only accept .heic, .jpg, .jpeg, .txt
        valid_img_exts = [".heic", ".jpg", ".jpeg"]
        if path.suffix.lower() in valid_img_exts:
            self.node.pending.setdefault(base, {})["img"] = path
        elif path.suffix.lower() == ".txt":
            self.node.pending.setdefault(base, {})["txt"] = path
        else:
            return

        files = self.node.pending.get(base, {})
        # Only process if exactly one image and one txt with the same name
        if "img" in files and "txt" in files and len(files) == 2:
            self.node.try_process(base)


def main():
    rclpy.init()
    node = SamClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SAM client...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
