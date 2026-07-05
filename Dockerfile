FROM ubuntu:22.04

LABEL maintainer="Amrit"
LABEL description="Omokai UAV Pipeline — ArduPilot SITL + LLM Mission Planner"

ENV DEBIAN_FRONTEND=noninteractive
ENV ANTHROPIC_API_KEY=""

# ── System dependencies ────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-dev git wget curl \
    # ArduPilot SITL dependencies
    gcc g++ make cmake ccache \
    libpython3-dev python3-numpy \
    # Gazebo Harmonic
    lsb-release gnupg2 \
    # Display / camera (for vision demo)
    libgl1-mesa-glx libglib2.0-0 \
    # MAVLink
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo Harmonic ────────────────────────────────────────────────────────
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ────────────────────────────────────────────────────────
COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

# ── ArduPilot SITL ─────────────────────────────────────────────────────────
RUN git clone --depth 1 --recurse-submodules \
    https://github.com/ArduPilot/ardupilot /opt/ardupilot \
    && cd /opt/ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && . ~/.profile \
    && ./waf configure --board sitl \
    && ./waf copter

# ── ardupilot_gazebo plugin ────────────────────────────────────────────────
RUN git clone --depth 1 \
    https://github.com/ArduPilot/ardupilot_gazebo /opt/ardupilot_gazebo \
    && cd /opt/ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j$(nproc)

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds

# ── Copy application ───────────────────────────────────────────────────────
WORKDIR /app
COPY . /app/

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["python3", "single_drone/main.py"]
