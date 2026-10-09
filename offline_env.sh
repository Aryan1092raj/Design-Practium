#!/usr/bin/env bash
# Source this in every sim/voice terminal so the sim and voice run with no internet:
#     source ~/wheelchair_nav/offline_env.sh
# Gazebo and Fast DDS discover each other by multicast. With Wi-Fi down, no interface can
# carry multicast ("Network is unreachable"), so the chair never spawns. This enables
# multicast on loopback (needs sudo once per boot) and pins Gazebo and Whisper to local use.

if ! ip link show lo | head -1 | grep -q MULTICAST; then
    sudo ip link set lo multicast on
fi
if ! ip route show | grep -q '^224.0.0.0/4'; then
    sudo ip route add 224.0.0.0/4 dev lo
fi

export GZ_IP=127.0.0.1                    # gz-transport discovery on loopback only
export HF_HUB_OFFLINE=1                   # faster-whisper uses the cached model only
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml
unset HTTP_PROXY HTTPS_PROXY NODE_USE_ENV_PROXY NO_PROXY no_proxy  # NO_PROXY "::1" crashes httpx
