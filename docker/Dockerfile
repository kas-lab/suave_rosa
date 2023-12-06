FROM ghcr.io/kas-lab/suave:main

COPY --chown=kasm-user:kasm-user ./ $HOME/suave_ws/src/suave_rosa
WORKDIR $HOME/suave_ws/src/suave_rosa/docker/
RUN ["/bin/bash", "-c", "./install_typedb.sh"]

WORKDIR $HOME/suave_ws
RUN ["/bin/bash", "-c", "vcs import src < src/suave_rosa/suave_rosa.rosinstall --force"]


RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y"]

# Build suave
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source ~/suave_ws/install/setup.bash' >> ~/.bashrc"]

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/
