FROM ros:foxy-ros1-bridge

RUN apt-get update \
        && apt-get install -q -y \
                libroscpp-dev \
        && apt-get clean -q -y \
        && apt-get autoremove -q -y \
        && rm -rf /var/lib/apt/lists/* \
        && git clone --branch Development https://github.com/5G-ERA/middleware \
        && mkdir -p /home/dev_ws/src/ \
        && cd /home/dev_ws/ \
        && cp -r /middleware/src/era_5g_action_client_ros2/ /home/dev_ws/src/ \
        && cp -r /middleware/src/era_5g_action_server_ros2/ /home/dev_ws/src/ \
        && cp -r /middleware/src/era_5g_action_interfaces_ros2/ /home/dev_ws/src/ \
        && sudo apt update \
        && sudo apt install -y python3-pip \
        && sudo pip install requests

        #&& . /opt/ros/melodic/setup.bash
        #&& cd /home/dev_ws/src \
        #&& catkin_init_workspace
RUN bash -c "cd /home/dev_ws/ && source /opt/ros/foxy/setup.bash && colcon build && source install/setup.bash"


RUN echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc && \echo "source /opt/ros/foxy/setup.bash" >> ~/.bashr