FROM robotis/ai-worker:2.2.8 AS dependencies
SHELL ["/bin/bash", "-c"]
# Build the checked-out source versions, without inheriting an older ROS overlay.
RUN rm -rf /root/ros2_ws
ENV COLCON_WS=/opt/internship_ws
WORKDIR /opt/internship_ws
COPY src/ai_worker/ src/ai_worker/
COPY src/cyclo_control/ src/cyclo_control/
COPY src/robotis_interfaces/ src/robotis_interfaces/
COPY src/realsense-ros/realsense2_camera_msgs/ src/realsense-ros/realsense2_camera_msgs/
COPY src/realsense-ros/realsense2_description/ src/realsense-ros/realsense2_description/
# ffw_bringup is used for model/controller assets; hardware teleoperation is not built.
RUN source /opt/ros/jazzy/setup.bash && MAKEFLAGS=-j2 colcon build \
    --executor sequential --base-paths src/ai_worker/ffw_description src/ai_worker/ffw_bringup \
    src/realsense-ros/realsense2_camera_msgs src/realsense-ros/realsense2_description \
    src/robotis_interfaces src/cyclo_control/osqp_eigen_vendor src/cyclo_control/cyclo_motion_controller_models \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN source install/setup.bash && MAKEFLAGS=-j2 colcon build \
    --executor sequential --base-paths src/cyclo_control/cyclo_motion_controller_core \
    src/cyclo_control/cyclo_motion_controller_ros src/cyclo_control/cyclo_motion_controller_ros_py \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM dependencies
# 공식 스워브 제어기도 현재 checkout 버전으로 빌드한다.
RUN source install/setup.bash && MAKEFLAGS=-j2 colcon build --executor sequential \
    --base-paths src/ai_worker/ffw_swerve_drive_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
COPY src/coop_carry_hardware/ src/coop_carry_hardware/
RUN source install/setup.bash && MAKEFLAGS=-j2 colcon build --executor sequential \
    --base-paths src/coop_carry_hardware \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
COPY src/coop_carry_bringup/ src/coop_carry_bringup/
RUN source install/setup.bash && colcon build --base-paths src/coop_carry_bringup
COPY docker/entrypoint.bash /usr/local/bin/coop-entrypoint
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=71 GZ_PARTITION=internship_dual
ENV ROS_LOG_DIR=/tmp/ros_logs
STOPSIGNAL SIGINT
ENTRYPOINT ["/bin/bash", "/usr/local/bin/coop-entrypoint"]
