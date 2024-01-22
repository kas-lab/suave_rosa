FROM suave:dev

RUN sudo apt update \
    && sudo apt install -y \
    openssh-client \
    && sudo rm -rf /var/lib/apt/list/

# Download public key for github.com
RUN USER=kasm-user mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

COPY --chown=kasm-user:kasm-user docker/install_typedb.sh $HOME/suave_ws/src/suave_rosa/docker/install_typedb.sh
WORKDIR $HOME/suave_ws/src/suave_rosa/docker/
RUN ["/bin/bash", "-c", "./install_typedb.sh"]

COPY --chown=kasm-user:kasm-user ./bts/ $HOME/suave_ws/src/suave_rosa/bts/
COPY --chown=kasm-user:kasm-user ./cmake/ $HOME/suave_ws/src/suave_rosa/cmake/
COPY --chown=kasm-user:kasm-user ./config/ $HOME/suave_ws/src/suave_rosa/config/
COPY --chown=kasm-user:kasm-user ./include/ $HOME/suave_ws/src/suave_rosa/include/
COPY --chown=kasm-user:kasm-user ./launch/ $HOME/suave_ws/src/suave_rosa/launch/
COPY --chown=kasm-user:kasm-user ./src/ $HOME/suave_ws/src/suave_rosa/src/
COPY --chown=kasm-user:kasm-user CMakeLists.txt $HOME/suave_ws/src/suave_rosa/CMakeLists.txt
COPY --chown=kasm-user:kasm-user package.xml $HOME/suave_ws/src/suave_rosa/package.xml
COPY --chown=kasm-user:kasm-user suave_rosa.rosinstall $HOME/suave_ws/src/suave_rosa/suave_rosa.rosinstall

WORKDIR $HOME/suave_ws
# RUN --mount=type=ssh,uid=1000 git clone git@github.com:kas-lab/rosa.git
RUN --mount=type=ssh,uid=1000 vcs import src < src/suave_rosa/suave_rosa.rosinstall --force


RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y"]

# Build suave
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source ~/suave_ws/install/setup.bash' >> ~/.bashrc"]

RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/

COPY --chown=kasm-user:kasm-user ./runner/ $HOME/suave_ws/src/suave/runner/
RUN chmod +x $HOME/suave_ws/src/suave/runner/rosa_runner.sh
RUN chmod +x $HOME/suave_ws/src/suave/runner/rosa_conf_results_run.sh
RUN chmod +x $HOME/suave_ws/src/suave/runner/scripts/launch_mission.sh
WORKDIR $HOME/suave_ws/src/suave/runner
