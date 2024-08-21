FROM ghcr.io/kas-lab/suave:main
# FROM suave:dev

RUN sudo apt update \
    && sudo apt install -y \
    openssh-client \
    && sudo rm -rf /var/lib/apt/list/

# Download public key for github.com
RUN USER=kasm-user mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

COPY --chown=kasm-user:kasm-user docker/install_typedb.sh $HOME/suave_ws/src/suave_rosa/docker/install_typedb.sh
WORKDIR $HOME/suave_ws/src/suave_rosa/docker/
RUN ["/bin/bash", "-c", "./install_typedb.sh"]

COPY --chown=kasm-user:kasm-user suave_rosa/ $HOME/suave_ws/src/suave_rosa/
COPY --chown=kasm-user:kasm-user suave_rosa_bt/ $HOME/suave_ws/src/suave_rosa_bt/
COPY --chown=kasm-user:kasm-user suave_rosa_plansys/ $HOME/suave_ws/src/suave_rosa_plansys/
COPY --chown=kasm-user:kasm-user suave_rosa.rosinstall $HOME/suave_ws/src/suave_rosa/suave_rosa.rosinstall

WORKDIR $HOME/suave_ws
RUN wget https://github.com/kas-lab/suave/blob/main/suave.rosinstall
RUN vcs import src < suave.rosinstall
RUN vcs import src < src/suave_rosa/suave_rosa.rosinstall --force


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
RUN chmod +x $HOME/suave_ws/src/suave/runner/rosa_paper_results_run.sh
RUN chmod +x $HOME/suave_ws/src/suave/runner/scripts/launch_mission.sh
WORKDIR $HOME/suave_ws/src/suave/runner
