# ROS 2 Jazzy dev image for botzo: headless (no RViz), visualise with Foxglove
# through foxglove_bridge, physics with Gazebo Harmonic (gz sim) server-only.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy python3-pytest python3-colcon-common-extensions \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-joy \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
