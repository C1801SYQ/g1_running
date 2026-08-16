#!/usr/bin/env bash
set -euo pipefail

LAN_INTERFACE="${ROBOT_LAN_INTERFACE:-enp4s0}"
WAN_INTERFACE="${ROBOT_WAN_INTERFACE:-Meta}"
ROBOT_SUBNET="${ROBOT_SUBNET:-192.168.5.0/24}"
ACTION="${1:-enable}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "This script must run as root." >&2
    exit 1
fi

add_rule() {
    local table="$1"
    local chain="$2"
    shift 2
    if ! iptables -t "${table}" -C "${chain}" "$@" 2>/dev/null; then
        iptables -t "${table}" -I "${chain}" 1 "$@"
    fi
}

delete_rule() {
    local table="$1"
    local chain="$2"
    shift 2
    while iptables -t "${table}" -C "${chain}" "$@" 2>/dev/null; do
        iptables -t "${table}" -D "${chain}" "$@"
    done
}

forward_out=(
    -i "${LAN_INTERFACE}"
    -o "${WAN_INTERFACE}"
    -s "${ROBOT_SUBNET}"
    -j ACCEPT
)
forward_back=(
    -i "${WAN_INTERFACE}"
    -o "${LAN_INTERFACE}"
    -d "${ROBOT_SUBNET}"
    -m conntrack
    --ctstate ESTABLISHED,RELATED
    -j ACCEPT
)
masquerade=(
    -s "${ROBOT_SUBNET}"
    -o "${WAN_INTERFACE}"
    -j MASQUERADE
)

case "${ACTION}" in
    enable)
        sysctl -w net.ipv4.ip_forward=1 >/dev/null
        add_rule filter FORWARD "${forward_out[@]}"
        add_rule filter FORWARD "${forward_back[@]}"
        add_rule nat POSTROUTING "${masquerade[@]}"
        echo "Robot NAT enabled: ${ROBOT_SUBNET} ${LAN_INTERFACE} -> ${WAN_INTERFACE}"
        ;;
    disable)
        delete_rule filter FORWARD "${forward_out[@]}"
        delete_rule filter FORWARD "${forward_back[@]}"
        delete_rule nat POSTROUTING "${masquerade[@]}"
        echo "Robot NAT rules removed."
        ;;
    status)
        sysctl net.ipv4.ip_forward
        iptables -C FORWARD "${forward_out[@]}"
        iptables -C FORWARD "${forward_back[@]}"
        iptables -t nat -C POSTROUTING "${masquerade[@]}"
        echo "Robot NAT rules are installed."
        ;;
    *)
        echo "Usage: $0 {enable|disable|status}" >&2
        exit 2
        ;;
esac
