#!/usr/bin/env bash
set -euo pipefail

PLC_IP="${PLC_IP:-192.168.1.88}"
HOST_IP="${HOST_IP:-192.168.1.10}"
PLC_IFACE="${PLC_IFACE:-enp7s0}"

sudo ip addr add "${HOST_IP}/24" dev "${PLC_IFACE}" 2>/dev/null || true
sudo ip route replace "${PLC_IP}/32" dev "${PLC_IFACE}" src "${HOST_IP}"
ip route get "${PLC_IP}"
ping -c 2 -W 1 "${PLC_IP}"
