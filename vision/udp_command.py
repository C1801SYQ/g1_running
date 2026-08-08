from __future__ import annotations

import socket

from .controller import ControllerCommand


class UdpCommandSender:
    """Sends an ASCII `vx vy wz` datagram to a local locomotion process."""

    def __init__(self, host: str = "127.0.0.1", port: int = 15001):
        self.address = (host, int(port))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command: ControllerCommand) -> None:
        payload = f"{command.vx:.6f} {command.vy:.6f} {command.wz:.6f}\n"
        self.socket.sendto(payload.encode("ascii"), self.address)

    def stop(self) -> None:
        self.socket.sendto(b"0.000000 0.000000 0.000000\n", self.address)

    def close(self) -> None:
        self.stop()
        self.socket.close()

    def __enter__(self) -> "UdpCommandSender":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


class UdpSkill6StatusReceiver:
    """Receives the actual C++ Skill 6 enabled/disabled state."""

    ENABLED_PAYLOAD = b"G1_VISION_SPRINT 1"
    DISABLED_PAYLOAD = b"G1_VISION_SPRINT 0"
    STATE1_PAYLOAD = b"G1_VISION_STATE1 1"

    def __init__(self, host: str = "127.0.0.1", port: int = 15002):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)
        self.address = self.socket.getsockname()
        self.enabled = False
        self.state1_entered = False

    def poll(self) -> bool:
        while True:
            try:
                payload, _address = self.socket.recvfrom(256)
            except BlockingIOError:
                break
            message = payload.strip()
            if message == self.ENABLED_PAYLOAD:
                self.enabled = True
            elif message == self.DISABLED_PAYLOAD:
                self.enabled = False
            elif message == self.STATE1_PAYLOAD:
                self.state1_entered = True
        return self.enabled

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> "UdpSkill6StatusReceiver":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
