#!/usr/bin/env bash
# Source after the system/Isaac ROS environment. No physical ROS domain is used.
RB20_PAINTING_DDS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# LOCALHOST discovery forcibly adds SHM transports in Jazzy rmw_fastrtps.
# Keep isolation in the explicit UDP interface whitelist instead.
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FASTDDS_DEFAULT_PROFILES_FILE="$RB20_PAINTING_DDS_ROOT/config/rb20_painting_fastdds.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
