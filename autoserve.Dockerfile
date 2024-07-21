# Base stage: Install dependencies
FROM ros:humble

ENV ROS_DISTRO="humble"

SHELL ["/bin/bash", "-c"]

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

RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros \
    ros-${ROS_DISTRO}-desktop-full

RUN apt-get update && apt upgrade -y

RUN apt-get update && apt-get install -y \
    gazebo

# Build stage: Copy source code and build
# FROM base as build

COPY /autoserve_3dmapping /colcon_ws/src/autoserve_3dmapping
COPY /autoserve_controller /colcon_ws/src/autoserve_controller
COPY /autoserve_description /colcon_ws/src/autoserve_description
COPY /autoserve_gazebo /colcon_ws/src/autoserve_gazebo
COPY /autoserve_mapping /colcon_ws/src/autoserve_mapping
COPY /autoserve_navigation /colcon_ws/src/autoserve_navigation
COPY /autoserve_perception /colcon_ws/src/autoserve_perception

WORKDIR /colcon_ws

RUN source /opt/ros/humble/setup.sh \
    && colcon build --executor sequential

# Runtime stage: Create the final image
# FROM ros:humble as runtime

# ENV ROS_DISTRO="humble"

# COPY --from=build /colcon_ws /colcon_ws
# COPY --from=base /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}
# COPY --from=base /etc/ros /etc/ros

COPY ./autoserve_entrypoint.bash /autoserve_entrypoint.bash

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

RUN echo "source /usr/share/gazebo/setup.bash" >> /etc/bash.bashrc

RUN echo "source /colcon_ws/install/setup.bash" >> /etc/bash.bashrc

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /etc/bash.bashrc

RUN chmod +x /autoserve_entrypoint.bash

ENTRYPOINT ["/autoserve_entrypoint.bash"]

CMD ["bash"]
