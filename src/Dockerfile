FROM ros:foxy-ros1-bridge



RUN apt-get update \
        && apt-get install -q -y \
        libroscpp-dev \
        && apt-get clean -q -y \
        && apt-get autoremove -q -y \
        && rm -rf /var/lib/apt/lists/* \
        && sudo apt-get update \
        && sudo apt-get install python3-pip -y \
        && mkdir -p home/dev_ws/src
COPY action_interfaces_5g /home/dev_ws/src
COPY middleware-actionserver-main /home/dev_ws/src
COPY middleware-actionClient-main /home/dev_ws/src
COPY Python_Interface_StandAlone /home/dev_ws/src
WORKDIR /home/dev_ws/
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash
RUN source /opt/ros/foxy/local_setup.bash
RUN source /opt/ros/foxy/setup.bash && colcon build --symlink-install

#COPY middleware-actionserver-main /home/dev_ws/src

#COPY action_interfaces_5g /home/dev_ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh
#RUN colcon build

#RUN cd /home/dev_ws
#RUN ls -l /home/dev_ws
WORKDIR /home/dev_ws/src
RUN pip install -r ActionServer5G/requirements.txt

RUN echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc && \
        echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

WORKDIR /home/dev_ws/

SHELL ["/bin/bash", "-c"]

WORKDIR /home/dev_ws/

SHELL ["/bin/bash", "-c"]

WORKDIR /home/dev_ws/

RUN source /opt/ros/foxy/setup.bash
RUN . /opt/ros/${ROS_DISTRO}/setup.sh
#RUN  colcon build


ENTRYPOINT [ "/bin/bash", "-c"]
CMD ["source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run ActionClient5G ActionClientNode" ]