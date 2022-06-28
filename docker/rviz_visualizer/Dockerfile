FROM ros:noetic

RUN apt-get -y update
RUN apt-get -y install ros-noetic-rviz
COPY . /home/movel_ws/src/rviz_visualizer

ENV QT_X11_NO_MITSHM 1

RUN bin/bash -c 'source /opt/ros/noetic/setup.bash && cd /home/movel_ws/ && catkin_make && rospack list'

RUN bin/bash -c  "source /home/movel_ws/devel/setup.bash"
RUN bin/bash -c "chmod +x /home/movel_ws/src/rviz_visualizer/entrypoint.sh"
ENTRYPOINT ["/home/movel_ws/src/rviz_visualizer/entrypoint.sh"]