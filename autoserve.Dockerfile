FROM ros:humble

ENV ROS_DISTRO="humble"

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-topic-tools-interfaces \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ament-* \
    ros-${ROS_DISTRO}-test-msgs \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-spatio-temporal-voxel-layer \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-gazebo* \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-octomap*

WORKDIR /colcon_ws/src

COPY /autoserve_3dmapping /autoserve_3dmapping

COPY /autoserve_controller /autoserve_controller

COPY /autoserve_description /autoserve_description

COPY /autoserve_gazebo /autoserve_gazebo

COPY /autoserve_mapping /autoserve_mapping

COPY /autoserve_navigation /autoserve_navigation

COPY /autoserve_perception /autoserve_perception

WORKDIR /colcon_ws

RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

COPY ./autoserve_entrypoint.bash /autoserve_entrypoint.bash

RUN chmod +x /autoserve_entrypoint.bash

ENTRYPOINT ["/autoserve_entrypoint.bash"]

CMD ["bash"]