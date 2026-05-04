import math
import socket
import struct


class FlightGearUdpPublisher:
    def __init__(self, host: str, port: int):
        self.host = str(host)
        self.port = int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @staticmethod
    def _pack_fdm_packet(
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> bytes:
        version = 24
        fmt = "!IIdddffff"
        header = struct.pack(
            fmt,
            version,
            0,
            math.radians(lon_deg),
            math.radians(lat_deg),
            float(alt_m),
            float(alt_m),
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )
        return header + (b"\x00" * (408 - len(header)))

    def publish(
        self,
        lat_deg: float,
        lon_deg: float,
        alt_m: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> None:
        packet = self._pack_fdm_packet(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, yaw_deg)
        self._sock.sendto(packet, (self.host, self.port))

    def close(self) -> None:
        self._sock.close()
