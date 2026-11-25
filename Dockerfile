FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-rf2o-laser-odometry \
        ros-humble-cv-bridge \
        python3-opencv \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --chmod=0644 ping.json /opt/go1/ping.json
COPY --chmod=0755 entrypoint.sh /entrypoint.sh
COPY --chmod=0755 src /opt/go1/src
RUN chmod -R 0755 /opt/go1

USER participant
ENTRYPOINT ["/entrypoint.sh"]
