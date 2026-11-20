FROM registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 AS sim_src

FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-rmw-cyclonedds-cpp \
       python3-pip \
       python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir mujoco==3.3.7 numpy \
    && useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY --from=sim_src /home/ros/go1_ws/install/unitree_go /opt/unitree_go

WORKDIR /opt/go1_ws
COPY safety_block /opt/go1_ws/src/safety_block

RUN bash -c "rm -rf /opt/go1_ws/src/safety_block/resource/safety_block \
    && mkdir -p /opt/go1_ws/src/safety_block/resource \
    && touch /opt/go1_ws/src/safety_block/resource/safety_block \
    && source /opt/ros/humble/setup.bash \
    && source /opt/unitree_go/share/unitree_go/local_setup.bash \
    && colcon build"

COPY models/unitree_go1 /opt/go1/models/unitree_go1
COPY ping.json /opt/go1/ping.json
RUN chmod -R a+rX /opt/go1 /opt/go1_ws /opt/unitree_go
COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
ENTRYPOINT ["/entrypoint.sh"]
