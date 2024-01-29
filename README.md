# suave_rosa
Applying ROSA to the SUAVE exemplar

Tested with ubuntu 22.04 and ROS Humble

## Install

Check the [SUAVE](https://github.com/kas-lab/suave) repo for additional installation instructions to install SUAVE.

Create workspace and clone repo:
```bash
mkdir -p ~/suave_rosa_ws/src
cd ~/suave_rosa_ws/src
git clone git@github.com:kas-lab/suave_rosa.git
```

Get dependencies:
```bash
source /opt/ros/humble/setup.bash
vcs import src < rosa_suave.rosinstall --recursive
rosdep install --from-paths src --ignore-src -r -y
```

Build:
```bash
colcon build --symlink-install
```

## Running

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

## Docker

The SUAVE docker image was extended to include ROSA. You can find additional information about the original SUAVE image in the SUAVE repo.

Run with docker:
```bash
docker run -it --shm-size=512m -v $HOME/rss_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave_rosa:main
```

You can run SUAVE + ROSA with the instructions in the [running section](##running)

Or you can use a runner to execute the experiments multiple times (check SUAVE repo for more info):
```bash
./rosa_runner.sh true rosa time 20
```
