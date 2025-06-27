# suave_rosa
Applying ROSA to the SUAVE exemplar

Tested with Ubuntu 22.04 and ROS Humble

## Install

Check the [SUAVE](https://github.com/kas-lab/suave) repo for additional installation instructions to install SUAVE.

Create a workspace and clone the repo:
```bash
mkdir -p ~/suave_rosa_ws/src
cd ~/suave_rosa_ws/src
git clone git@github.com:kas-lab/suave_rosa.git
```

Get dependencies:
```bash
source /opt/ros/humble/setup.bash
cd ~/suave_rosa_ws/
wget https://raw.githubusercontent.com/kas-lab/suave/refs/heads/main/suave.repos
vcs import src < suave.repos
vcs import src < src/suave_rosa/suave_rosa.repos --recursive
rosdep install --from-paths src --ignore-src -r -y
```

Build:
```bash
colcon build --symlink-install
```

## Docker

The SUAVE docker images were extended to include ROSA. You can find additional information about the original SUAVE image in the SUAVE repo.

### Docker with web interface

Run with Docker:
```bash
docker run -it --shm-size=512m -v $HOME/suave_rosa_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave_rosa:main
```

In your browser go the address: `http://localhost:6901`
Login:
    - **User** : `kasm_user`
    - **Password**: `password`

You can run SUAVE + ROSA with the instructions in the [running section](##running)

### Docker headless

Run docker image without web interface (with nvidia)(don't forget to install the [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html))

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_rosa_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave_rosa-headless:latest
```

Run docker image without web interface (without nvidia):
```Bash
docker run -it --rm --name suave_rosa_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave_rosa-headless:latest
```

**DEV**:

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_rosa_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave_rosa:/home/ubuntu-user/suave_ws/src/suave_rosa suave_rosa
```

### Build Docker images locally

Docker image with web interface:
```Bash
docker build -t suave_rosa -f docker/Dockerfile .
```

Docker image without web interface:
```Bash
docker build -t suave_rosa-headless -f docker/Dockerfile-headless .
```


## Running manually

Run ardusub SITL:
```bash
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

Run SUAVE simulation:
```bash
source ~/suave_rosa_ws/install/setup.bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

Run ROSA for the original SUAVE use case:
```bash
source ~/suave_rosa_ws/install/setup.bash
ros2 launch suave_rosa suave_rosa.launch.py mission_type:=time_constrained_mission result_filename:=suave_rosa_result
```

Run ROSA for the extended SUAVE use case:
```bash
source ~/suave_rosa_ws/install/setup.bash
ros2 launch suave_rosa suave_rosa_extended.launch.py mission_type:=time_constrained_mission result_filename:=suave_rosa_result
```

## Runner

### Python runner (most updated)

You can run it with the launchfile:

```Bash
ros2 launch suave_rosa_runner suave_runner_launch.py
```

**Changing experiments config:** Simply change the [runner_config.yml](suave_rosa_runner/config/runner_config.yml) file

Or you can run it with the suave_runner ros node:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_rosa_bt suave_rosa_bt.launch.py\", \
      \"num_runs\": 20, \
      \"adaptation_manager\": \"rosa_bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_rosa_bt suave_rosa_extended_bt.launch.py\", \
      \"num_runs\": 20, \
      \"adaptation_manager\": \"rosa_bt\", \
      \"mission_name\": \"suave_extended\"}"
  ]'
```

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_rosa_plansys suave_rosa_plansys.launch.py\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"rosa_plansys\", \
      \"mission_name\": \"suave\"}"
  ]'
```

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_rosa_plansys suave_rosa_extended_plansys.launch.py\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"rosa_plansys\", \
      \"mission_name\": \"suave_extended\"}"
  ]'
```

### Bash runner (only works with the docker image with the web interface)

You can use a runner to execute the experiments multiple times (check SUAVE repo for more info):
```bash
./rosa_runner.sh true rosa time 20
```
