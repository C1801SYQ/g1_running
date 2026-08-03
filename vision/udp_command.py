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
