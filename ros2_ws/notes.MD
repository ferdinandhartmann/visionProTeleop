ros2 launch mycobot_280 slider_control_adaptive_gripper.launch.py 

chmod +x teleoperation/joint_state_to_mycobot.py or other python executables if not found

pip install mujoco-usd-converter


pip install icloudpd

icloudpd --directory ~/visionpro_teleop_project/visionProTeleop/ros2_ws/icloud_data --username ferdinand.hartmann@web.de --watch-with-interval 15

https://github.com/icloud-photos-downloader/icloud_photos_downloader?tab=readme-ov-file


if changing somehthing in apv_streamer files install with pip install -e . instead of pip --upgrade

ffmpeg -i disabled_sound.mp3 -ac 1 -ar 48000 disabled.wav


Test internet connection speed: 

ping -c 200 -i 0.02 IP_ADRESS

Ferdinand Hotspot:
--- 192.0.0.2 ping statistics ---
200 packets transmitted, 200 received, 0% packet loss, time 4274ms
rtt min/avg/max/mdev = 1.702/30.720/169.202/42.339 ms, pipe 6

not low data mode and still maximise compability:
--- 192.0.0.2 ping statistics ---
200 packets transmitted, 200 received, 0% packet loss, time 4108ms
rtt min/avg/max/mdev = 1.582/6.077/82.202/9.676 ms, pipe 4


Plume WIFI:
--- 192.168.10.113 ping statistics ---
200 packets transmitted, 200 received, 0% packet loss, time 4148ms
rtt min/avg/max/mdev = 3.713/64.901/348.065/77.993 ms, pipe 17


JEMARO Room Wifi:
--- 192.168.50.153 ping statistics ---
200 packets transmitted, 200 received, 0% packet loss, time 4078ms
rtt min/avg/max/mdev = 2.985/4.393/10.271/1.091 ms
