FROM ghcr.io/kas-lab/suave:main
# FROM suave:dev

RUN sudo apt update \
    && sudo apt install -y \
    openssh-client \
    && sudo rm -rf /var/lib/apt/list/

# Download public key for github.com
RUN USER=kasm-user mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

## Install TypeDB
RUN sudo gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 17507562824cfdcc && \
    sudo gpg --export 17507562824cfdcc | sudo tee /etc/apt/trusted.gpg.d/vaticle.gpg > /dev/null && \
    echo "deb https://repo.typedb.com/public/public-release/deb/ubuntu trusty main" | sudo tee /etc/apt/sources.list.d/vaticle.list > /dev/null

RUN sudo apt-get update && sudo apt-get install -y \
    typedb=2.29.1 \
    && sudo rm -rf /var/lib/apt/lists/*

RUN pip3 install typedb-driver==2.29.7

COPY --chown=kasm-user:kasm-user suave_rosa/ $HOME/suave_ws/src/suave_rosa/
COPY --chown=kasm-user:kasm-user suave_rosa_bt/ $HOME/suave_ws/src/suave_rosa_bt/
COPY --chown=kasm-user:kasm-user suave_rosa_plansys/ $HOME/suave_ws/src/suave_rosa_plansys/
COPY --chown=kasm-user:kasm-user suave_rosa.repos $HOME/suave_ws/src/suave_rosa/suave_rosa.repos
COPY --chown=ubuntu-user:ubuntu-user suave_rosa_runner/ $HOME/suave_ws/src/suave_rosa_runner/

WORKDIR $HOME/suave_ws
RUN wget https://raw.githubusercontent.com/kas-lab/suave/refs/heads/main/suave.repos
RUN vcs import src < suave.repos
RUN vcs import src < src/suave_rosa/suave_rosa.repos --force


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
