FROM ros:humble
SHELL ["/bin/bash", "-c"]

# China mirror for Ubuntu apt (git clones go direct to GitHub to get the current SDK)
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' /etc/apt/sources.list

# Drop the GFW-blocked ROS apt source (ROS is already in the image); don't let a failed source abort the build
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.sources ; \
    apt-get update || true ; \
    apt-get install -y build-essential cmake pkg-config git python3 && rm -rf /var/lib/apt/lists/*

# YDLidar-SDK (direct from GitHub -> current version with the T-mini Plus fix)
RUN git clone --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git /opt/YDLidar-SDK && \
    cd /opt/YDLidar-SDK && mkdir build && cd build && cmake .. && make -j"$(nproc)" && make install && ldconfig

# ydlidar_ros2_driver, patched for Humble + T-mini Plus (intensity off)
WORKDIR /root/lidar_ws/src
RUN git clone --depth 1 https://github.com/YDLIDAR/ydlidar_ros2_driver.git
COPY fix_declare_params.py /root/lidar_ws/src/ydlidar_ros2_driver/fix_declare_params.py
WORKDIR /root/lidar_ws/src/ydlidar_ros2_driver
RUN python3 fix_declare_params.py && \
    sed -i -e 's/node_executable/executable/g' -e 's/node_namespace/namespace/g' -e 's/node_name/name/g' launch/ydlidar_launch.py && \
    cp params/TminiPro.yaml params/ydlidar.yaml && \
    sed -i 's/intensity: true/intensity: false/' params/ydlidar.yaml

WORKDIR /root/lidar_ws
RUN source /opt/ros/humble/setup.bash && colcon build

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /root/lidar_ws/install/setup.bash' >> /root/.bashrc
WORKDIR /root
CMD ["bash"]
