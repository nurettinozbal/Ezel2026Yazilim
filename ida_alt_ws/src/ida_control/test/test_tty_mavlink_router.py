import socket
import unittest
from unittest.mock import patch

from ida_control.tty_mavlink_router import (
    forward_ready,
    open_endpoint_sockets,
    validate_config,
)


class TtyMavlinkRouterTests(unittest.TestCase):
    def test_accepts_vehicle_defaults(self):
        self.assertEqual(
            validate_config("/dev/ttyACM0", "115200", ["14540", "14541"]),
            ("/dev/ttyACM0", 115200, (14540, 14541)),
        )

    def test_rejects_network_paths_bad_baud_and_duplicate_ports(self):
        for args in (
            ("udp://0.0.0.0:14550", "115200", ["14540", "14541"]),
            ("/dev/ttyACM0", "115200.0", ["14540", "14541"]),
            ("/dev/ttyACM0", "12345", ["14540", "14541"]),
            ("/dev/ttyACM0", "115200", ["14540", "14540"]),
            ("/dev/ttyACM0", "115200", ["80"]),
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                validate_config(*args)

    def test_endpoint_sockets_bind_only_loopback(self):
        endpoints = open_endpoint_sockets((14540, 14541))
        try:
            self.assertEqual(len(endpoints), 2)
            for sock, target in endpoints:
                self.assertEqual(sock.getsockname()[0], "127.0.0.1")
                self.assertEqual(target[0], "127.0.0.1")
        finally:
            for sock, _target in endpoints:
                sock.close()

    def test_partial_socket_failure_closes_already_opened_socket(self):
        first = unittest.mock.MagicMock(spec=socket.socket)
        with patch("socket.socket", side_effect=[first, OSError("bind failed")]):
            with self.assertRaises(OSError):
                open_endpoint_sockets((14540, 14541))
        first.close.assert_called_once()

    def test_serial_bytes_fan_out_to_both_local_clients(self):
        pixhawk = unittest.mock.MagicMock()
        pixhawk.read.return_value = b"mavlink-frame"
        first = unittest.mock.MagicMock()
        second = unittest.mock.MagicMock()
        endpoints = [
            (first, ("127.0.0.1", 14540)),
            (second, ("127.0.0.1", 14541)),
        ]
        forward_ready(pixhawk, endpoints, [pixhawk])
        first.sendto.assert_called_once_with(b"mavlink-frame", endpoints[0][1])
        second.sendto.assert_called_once_with(b"mavlink-frame", endpoints[1][1])

    def test_only_loopback_client_bytes_reach_pixhawk(self):
        pixhawk = unittest.mock.MagicMock()
        local = unittest.mock.MagicMock()
        local.recvfrom.return_value = (b"local-command", ("127.0.0.1", 50000))
        remote = unittest.mock.MagicMock()
        remote.recvfrom.return_value = (b"remote-command", ("192.168.11.10", 50001))
        endpoints = [
            (local, ("127.0.0.1", 14540)),
            (remote, ("127.0.0.1", 14541)),
        ]
        forward_ready(pixhawk, endpoints, [local, remote])
        pixhawk.write.assert_called_once_with(b"local-command")


if __name__ == "__main__":
    unittest.main()
