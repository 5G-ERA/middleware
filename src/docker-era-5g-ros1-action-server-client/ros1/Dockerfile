FROM ros:melodic-ros-base

RUN apt-get update \
        && apt-get install -q -y \
                ros-melodic-ros-tutorials \
        && apt-get clean -q -y \
        && apt-get autoremove -q -y \
        && rm -rf /var/lib/apt/lists/* \
        && git clone --branch Development https://github.com/5G-ERA/middleware \
        && mkdir -p /home/dev_ws/src/ \
        && cd /home/dev_ws/src \
 		  && cp -r /middleware/src/era_5g_action_client_ros1/ /home/dev_ws/src/ \
        && cp -r /middleware/src/era_5g_action_server_ros1/ /home/dev_ws/src/ \
        && cp -r /middleware/src/era_5g_action_interfaces/ /home/dev_ws/src/ \
        && sudo apt update \
        && sudo apt install -y python3-pip \
        && sudo pip3 install requests


        #&& . /opt/ros/melodic/setup.bash
        #&& cd /home/dev_ws/src \
        #&& catkin_init_workspace
RUN bash -c "cd /home/dev_ws/src && source /opt/ros/melodic/setup.bash && catkin_init_workspace"

#RUN bash -c "cp -r /middleware/src/middleware-actionClient-main /home/dev_ws/src/"


RUN echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc && \
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
