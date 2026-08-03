import math
import os
from argparse import ArgumentTypeError
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import gestalt_bridge_windows as bridge  # noqa: E402


class FakeControl:
    def __init__(self):
        self.aim_value = None
        self.clear_count = 0
        self.updated = threading.Event()

    def aim(self, yaw, pitch, fire, yaw_velocity=0.0, pitch_velocity=0.0):
        self.aim_value = (yaw, pitch, fire, yaw_velocity, pitch_velocity)
        self.updated.set()

    def clear_fire(self):
        self.clear_count += 1


class GestaltWindowsProtocolTest(unittest.TestCase):
    def test_ws_port_and_process_name_parsing(self):
        self.assertEqual(bridge.parse_ws_port("auto"), 0)
        self.assertEqual(bridge.parse_ws_port("49152"), 49152)
        with self.assertRaises(ArgumentTypeError):
            bridge.parse_ws_port("90000")
        self.assertTrue(bridge.is_gestalt_process_name(
            r"E:\\GestaltSystem\\RobotBridgeDemo-Win64-Shipping.exe"))
        self.assertFalse(bridge.is_gestalt_process_name("python.exe"))

    def test_render_rate_argument_defaults_to_sixty_and_is_configurable(self):
        with mock.patch.object(sys, "argv", ["gestalt_bridge_windows.py"]):
            arguments = bridge.parse_arguments()
            self.assertEqual(arguments.max_fps, 60.0)
            self.assertEqual(arguments.frame_codec, "lz4")
        with mock.patch.object(
                sys, "argv", ["gestalt_bridge_windows.py", "--max-fps", "0"]):
            self.assertEqual(bridge.parse_arguments().max_fps, 0.0)

    def test_match_spawn_identity_gate(self):
        control = bridge.GameControl.__new__(bridge.GameControl)
        control.player_id = 0
        control.entity_id = 66000005
        control.team_id = 0
        control._maps = {}

        payload = {
            "type": 0,
            "method": "watchAttributeMaps.result",
            "params": {
                "watch_attribute_maps_results": [
                    {
                        "attribute_map_id": 1,
                        "attributes": {
                            "0": 10,
                            str(bridge.A_MATCH_STATUS): 0,
                        },
                    },
                    {
                        "attribute_map_id": 10,
                        "attributes": {
                            str(bridge.A_MAP_PTR): 20,
                            str(bridge.A_CONNECTION_ENTITY_CONFIG_ID): 66000005,
                        },
                    },
                    {
                        "attribute_map_id": 20,
                        "attributes": {
                            str(bridge.A_HEALTH): 500,
                            str(bridge.A_PLAYER_ID): 0,
                            str(bridge.A_TEAM_ID): 0,
                            str(bridge.A_CLASS): 1004,
                        },
                    },
                ],
            },
        }
        discovered = control._update_telemetry(payload)
        self.assertEqual(discovered, {10, 20})
        self.assertEqual(control._match_status(), 0)
        self.assertEqual(control._expected_spawn_map(19), 20)

        control._maps[20][bridge.A_TEAM_ID] = 1
        self.assertIsNone(control._expected_spawn_map(19))

    def test_wire_sizes(self):
        self.assertEqual(bridge.ENVELOPE.size, 32)
        self.assertEqual(bridge.FRAME_METADATA.size, 148)
        self.assertEqual(bridge.COMMAND_PACKET.size, 112)
        self.assertEqual(bridge.REGION_PREFIX.size, 64)

    def test_fragmented_command_and_coordinate_conversion(self):
        sender, receiver = socket.socketpair()
        control = FakeControl()
        stop = threading.Event()
        worker = threading.Thread(
            target=bridge.command_loop,
            args=(receiver, control, stop, 1.0), daemon=True)
        worker.start()

        sequence = 7
        command = bridge.COMMAND_PACKET.pack(
            b"HFUTCMD1", 1, 0, sequence, 12.0,
            math.pi / 2.0, 0.0, math.pi / 4.0, 0.0,
            math.pi / 6.0, 0.0, math.pi / 12.0, 0.0,
            4.0, 1, 1, 0, 0)
        envelope = bridge.ENVELOPE.pack(
            bridge.ENVELOPE_MAGIC, bridge.ENVELOPE_VERSION,
            bridge.MESSAGE_COMMAND, len(command), sequence)
        message = envelope + command
        sender.sendall(message[:13])
        sender.sendall(message[13:41])
        sender.sendall(message[41:])

        self.assertTrue(control.updated.wait(1.0))
        self.assertAlmostEqual(control.aim_value[0], -90.0)
        self.assertAlmostEqual(control.aim_value[1], 30.0)
        self.assertTrue(control.aim_value[2])
        self.assertAlmostEqual(control.aim_value[3], -45.0)
        self.assertAlmostEqual(control.aim_value[4], 15.0)

        stop.set()
        sender.close()
        receiver.close()
        worker.join(timeout=1.0)

    def test_stdio_command_and_coordinate_conversion(self):
        read_fd, write_fd = os.pipe()
        input_stream = os.fdopen(read_fd, "rb", buffering=0)
        control = FakeControl()
        stop = threading.Event()
        worker = threading.Thread(
            target=bridge.stdio_command_loop,
            args=(input_stream, control, stop, 1.0), daemon=True)
        worker.start()

        sequence = 8
        command = bridge.COMMAND_PACKET.pack(
            b"HFUTCMD1", 1, 0, sequence, 12.0,
            math.pi / 2.0, 0.0, math.pi / 4.0, 0.0,
            math.pi / 6.0, 0.0, math.pi / 12.0, 0.0,
            4.0, 1, 1, 0, 0)
        envelope = bridge.ENVELOPE.pack(
            bridge.ENVELOPE_MAGIC, bridge.ENVELOPE_VERSION,
            bridge.MESSAGE_COMMAND, len(command), sequence)
        os.write(write_fd, envelope[:11])
        os.write(write_fd, envelope[11:] + command)

        self.assertTrue(control.updated.wait(1.0))
        self.assertAlmostEqual(control.aim_value[0], -90.0)
        self.assertAlmostEqual(control.aim_value[1], 30.0)
        self.assertAlmostEqual(control.aim_value[3], -45.0)
        self.assertAlmostEqual(control.aim_value[4], 15.0)

        stop.set()
        os.close(write_fd)
        worker.join(timeout=1.0)
        input_stream.close()

    def test_frame_packet_layout(self):
        frame = {
            "seq": 3,
            "capture_time_s": 1.5,
            "world_time_s": 1.0,
            "width": 1,
            "height": 1,
            "row_bytes": 4,
            "pixel_bytes": 4,
            "pixel_format": 1,
            "fov_degrees": 45.0,
            "camera_arm_length_cm": 0.0,
            "camera_position": (0.0, 0.0, 0.0),
            "camera_quaternion": (0.0, 0.0, 0.0, 1.0),
            "writer_pid": 100,
            "view_actor": 5,
            "takeover_target": 5,
            "takeover_player": 0,
            "takeover_map": 6,
            "takeover_epoch": 2,
            "identity_flags": 7,
            "pixels": b"\x01\x02\x03\xff",
        }
        packet = bridge.frame_packet(frame)
        envelope = bridge.ENVELOPE.unpack_from(packet)
        self.assertEqual(envelope[2], bridge.MESSAGE_FRAME)
        self.assertEqual(envelope[3], bridge.FRAME_METADATA.size + 4)
        self.assertEqual(len(packet), bridge.ENVELOPE.size + envelope[3])

        compressible_frame = dict(frame)
        compressible_frame.update({
            "width": 256,
            "height": 1,
            "row_bytes": 1024,
            "pixel_bytes": 1024,
            "pixels": b"\x00\x00\x00\xff" * 256,
        })
        compressed_packet = bridge.frame_packet(compressible_frame, "lz4")
        compressed_envelope = bridge.ENVELOPE.unpack_from(compressed_packet)
        metadata_offset = bridge.ENVELOPE.size
        metadata = bridge.FRAME_METADATA.unpack_from(compressed_packet, metadata_offset)
        self.assertEqual(
            metadata[6], bridge.PIXEL_FORMAT_BGRA8 + bridge.PIXEL_FORMAT_LZ4_OFFSET)
        self.assertLess(metadata[7], compressible_frame["pixel_bytes"])
        payload_offset = metadata_offset + bridge.FRAME_METADATA.size
        import lz4.block
        decoded = lz4.block.decompress(
            compressed_packet[payload_offset:],
            uncompressed_size=compressible_frame["pixel_bytes"])
        self.assertEqual(decoded, compressible_frame["pixels"])
        self.assertEqual(
            len(compressed_packet), bridge.ENVELOPE.size + compressed_envelope[3])

    def test_frame_contract(self):
        arguments = SimpleNamespace(
            no_identity_gate=False,
            player_id=0,
            frame_width=1280,
            frame_height=720,
            fov=25.0,
            arm_length=0.0,
        )
        frame = {
            "identity_flags": 7,
            "takeover_player": 0,
            "takeover_map": 42,
            "view_actor": 9,
            "takeover_target": 9,
            "takeover_epoch": 3,
            "width": 1280,
            "height": 720,
            "fov_degrees": 25.0,
            "camera_arm_length_cm": 0.0,
        }
        self.assertIsNone(bridge.frame_rejection_reason(frame, arguments))
        frame["camera_arm_length_cm"] = 400.0
        self.assertIn("arm=400.00cm", bridge.frame_rejection_reason(frame, arguments))


if __name__ == "__main__":
    unittest.main()
