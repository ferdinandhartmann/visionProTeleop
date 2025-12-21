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

class SamClient(Node):

    def __init__(self):
        super().__init__("sam_client")
        self.get_logger().info("SAM client started")

        self.pending = {}  # basename -> files

        # Load parameters
        self.declare_parameter("watch_dir", str(Path.home() / "Dropbox/sam_data"))
        self.declare_parameter("server_url", "http://localhost:18000")
        self.declare_parameter("result_files", ["model1.obj", "model2.obj", "material.mtl", "texture.png"])

        self.watch_dir = Path(self.get_parameter("watch_dir").value)
        self.server_url = self.get_parameter("server_url").value
        self.result_files = self.get_parameter("result_files").value

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
                time.sleep(3)
                continue
            self.get_logger().info(f"Job status: {data['status']}")
            if data["status"] == "done":
                self.get_logger().info(f"Job {job_id} done, downloading results")
                self.download_results(job_id)
                break
            time.sleep(3)

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
            self.get_logger().info(f"Downloading {fname} for job {job_id}")
            r = requests.get(f"{self.server_url}/download/{job_id}/{fname}", verify=False)
            # Save to both folders
            (install_out_dir / fname).write_bytes(r.content)
            (workspace_src_dir / fname).write_bytes(r.content)
            self.get_logger().info(f"Saved {fname} to {install_out_dir} and {workspace_src_dir}")

        self.get_logger().info("All results downloaded")


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
