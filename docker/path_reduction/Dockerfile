FROM ros:noetic

COPY . /home/movel_ws/src/path_reduction
RUN bin/bash -c 'chmod +x /home/movel_ws/src/path_reduction/script/*'
RUN bin/bash -c 'source /opt/ros/noetic/setup.bash && cd /home/movel_ws/ && catkin_make && rospack list'

RUN bin/bash -c  "source /home/movel_ws/devel/setup.bash"
RUN bin/bash -c "chmod +x /home/movel_ws/src/path_reduction/entrypoint.sh"
ENTRYPOINT ["/home/movel_ws/src/path_reduction/entrypoint.sh"]