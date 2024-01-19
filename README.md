# suave_rosa
Applying ROSA to the SUAVE exemplar

## Install

```
source /opt/ros/humble/setup.bash
vcs import src < rosa_suave.rosinstall --recursive
rosdep install --from-paths src --ignore-src -r -y
```

## Running

## docker Build

docker build --ssh default -t suave_rosa -f docker/Dockerfile .
