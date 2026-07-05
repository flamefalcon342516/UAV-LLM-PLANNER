FROM ubuntu:22.04

LABEL maintainer="Amrit"
LABEL description="Omokai UAV Pipeline — ArduPilot SITL (no Gazebo) + LLM Mission Planner. Single-drone and multi-UAV swarm only."

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# ── System dependencies ────────────────────────────────────────────────────
# tzdata is pulled in transitively by several packages below; pre-seed it
# non-interactively here so it's already configured before install-prereqs
# (run later via sudo, which strips DEBIAN_FRONTEND) ever touches it.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    python3 python3-pip python3-dev git wget curl sudo \
    # ArduPilot SITL build toolchain
    gcc g++ make cmake ccache \
    libpython3-dev python3-numpy \
    # tmux — needed by multi_uav/sim/launch_swarm_sitl.sh
    tmux \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ────────────────────────────────────────────────────────
COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt \
    && pip3 install --no-cache-dir MAVProxy

# ── Non-root user for the ArduPilot build ──────────────────────────────────
# install-prereqs-ubuntu.sh refuses to run as root ("don't sudo it!").
RUN useradd -m -s /bin/bash builder \
    && { echo "builder ALL=(ALL) NOPASSWD:ALL"; \
         echo 'Defaults:builder env_keep += "DEBIAN_FRONTEND TZ"'; \
       } > /etc/sudoers.d/builder \
    && chmod 0440 /etc/sudoers.d/builder \
    && mkdir -p /opt/ardupilot && chown builder:builder /opt/ardupilot

# ── ArduPilot SITL (ArduCopter only, no hardware/STM32 toolchain) ──────────
USER builder
ENV USER=builder
ENV HOME=/home/builder
# SITL only — skip the STM32 cross-compiler download (real-hardware builds only).
ENV DO_AP_STM_ENV=0
RUN git clone --depth 1 --recurse-submodules \
    https://github.com/ArduPilot/ardupilot /opt/ardupilot \
    && cd /opt/ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && . ~/.profile \
    && ./waf configure --board sitl \
    && ./waf copter
USER root

ENV PATH="/opt/ardupilot/Tools/autotest:${PATH}"

# ── Copy application ───────────────────────────────────────────────────────
WORKDIR /app
COPY . /app/

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["python3", "single_drone/main.py"]
