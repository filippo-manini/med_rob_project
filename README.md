# Med Rob Project

Containerized ROS 2 Humble environment for working with the **NDI Aurora** electromagnetic tracker.
Everything runs inside Docker: you do not need to install ROS on your computer.

## Repository contents

```
med_rob_project/
├── Dockerfile              # ROS 2 Humble image + tools
├── docker-compose.yml      # container configuration
├── setup.sh                # downloads the Aurora driver and builds (run once)
├── aurora_overrides/       # config and launch files customized for this course
└── src/
    └── robot_control/      # the project code
```

The Aurora driver (`aurora_ndi_ros2_driver`) is **not in this repo**: `setup.sh` downloads it.

## Prerequisites

- Linux (tested on Ubuntu) with **Docker 20.10+** and **Docker Compose 2.0+**
- Aurora tracker connected via USB (`/dev/ttyUSB0`)

Install Docker if you don't have it:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh
sudo usermod -aG docker $USER
sudo apt-get install docker-compose-plugin
# Log out and back in
```

## First-time setup

**1. On your computer**, clone the repo and start the container:

```bash
git clone https://github.com/filippo-manini/med_rob_project.git
cd med_rob_project
xhost +local:root          # needed for GUI windows (RViz)
docker compose up -d --build
```

The first build takes a few minutes.

**2. Enter the container:**

```bash
docker exec -it med_rob_dev bash
```

From here on, commands must be run **inside the container** (the prompt looks like `root@...:/workspace#`).

**3. Download the driver and build:**

```bash
bash setup.sh
source install/setup.bash
```

The script clones the Aurora driver, applies the files in `aurora_overrides/`, and runs `colcon build`.

## Daily use

Start the container (if it is not already running) and open a shell:

```bash
cd med_rob_project
docker compose up -d
docker exec -it med_rob_dev bash
```

Launch the driver:

```bash
ros2 launch aurora_ndi_ros2_driver aurora_tracking.launch.py
```

In a **second shell** (open another terminal and run `docker exec -it med_rob_dev bash` again) you can inspect the data:

```bash
ros2 topic list
ros2 topic echo /aurora/sensor0
ros2 topic hz /aurora/sensor0
```

Published topics:

| Topic | Content |
|-------|---------|
| `/aurora/sensor0` | raw sensor data (`AuroraData`) |
| `/aurora/sensor0/kalman_filter` | Kalman-filtered data |
| `/aurora/sensor0/lowpass_filter` | low-pass-filtered data |
| `/tf` | transforms (`world` → `aurora_base` → sensor) |

To stop the container when you are done:

```bash
docker compose down
```

## Developing your own code

Put your code in `src/<package_name>/`. The project folder is shared between your computer and the container (`/workspace`), so you can edit files with your own editor on the host and build inside the container:

```bash
cd /workspace
colcon build --packages-select robot_control
source install/setup.bash
```

To use the tracker data in your code:

```python
from aurora_ndi_ros2_driver.msg import AuroraData
```

and add `aurora_ndi_ros2_driver` as a dependency in your package's `package.xml`.

## Run `ros2` commands only inside the container

Do not run `ros2` directly on your computer: if it has a different ROS version, it will not find the packages and will fail with errors like `package 'aurora_ndi_ros2_driver' not found`. Always enter the container with `docker exec -it med_rob_dev bash`.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Not able to open: /dev/ttyUSB0` | The tracker is not connected or has a different name. Check with `ls -l /dev/ttyUSB*`, then recreate the container (`docker compose down && docker compose up -d`) |
| `docker compose up` fails on the device | Connect the tracker before starting, or comment out the `devices:` section in `docker-compose.yml` to work without hardware |
| `./setup.sh: Permission denied` | Use `bash setup.sh` |
| GUI windows (RViz) do not open | Run `xhost +local:root` on the host and recreate the container |
| `package 'aurora_ndi_ros2_driver' not found` | You are outside the container, or you did not run `source install/setup.bash` |
| `message type ... is invalid` | Missing `source install/setup.bash` in the current shell |
| Cannot delete `build/`, `install/`, `log/` from the host | They were created by the container as root: `sudo rm -rf build install log` |
| Changes in `src/` not picked up | Rebuild with `colcon build` and run `source install/setup.bash` again |

## Rebuilding from scratch

```bash
# inside the container
cd /workspace
rm -rf build install log
bash setup.sh
source install/setup.bash
```

To also rebuild the Docker image (on the host):

```bash
docker compose down
docker compose build --no-cache
docker compose up -d
```

## Notes

- The driver is the [`aurora_ndi_ros2_driver`](https://github.com/eddrive/aurora_ndi_ros2_driver) package (branch `ros2-package`), developed at NEARlab, Politecnico di Milano.
- The files in `aurora_overrides/` replace the originals in the driver. To change the configuration (ROM files, `port_handles`, filters), edit `aurora_overrides/aurora_tracking_config.yaml`, then run `bash setup.sh` again.
