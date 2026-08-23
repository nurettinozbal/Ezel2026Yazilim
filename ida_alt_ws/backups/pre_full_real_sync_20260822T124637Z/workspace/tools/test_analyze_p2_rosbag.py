import json
import sqlite3
import struct
import tempfile
import unittest
from pathlib import Path

from analyze_p2_rosbag import analyze_bag, decode_std_string


def cdr_string(payload):
    encoded = json.dumps(payload, separators=(",", ":")).encode() + b"\0"
    return b"\x00\x01\x00\x00" + struct.pack("<I", len(encoded)) + encoded


class AnalyzeP2BagTests(unittest.TestCase):
    def make_bag(self, root):
        path = Path(root) / "run.db3"
        connection = sqlite3.connect(path)
        connection.executescript(
            "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT);"
            "CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);"
        )
        names = ["/autonomy/state", "/autonomy/debug", "/telemetry/state", "/perception/fusion/status"]
        for index, name in enumerate(names, 1):
            connection.execute("INSERT INTO topics VALUES(?,?,?,?,?)", (index, name, "std_msgs/msg/String", "cdr", ""))
        states = [
            {"state": "PARKUR_2_AVOIDANCE", "distance_from_course_m": 1.0, "inside_course_geometry": True},
            {"state": "PARKUR_2_AVOIDANCE", "distance_from_course_m": 6.0, "inside_course_geometry": False},
            {"state": "PARKUR_2_AVOIDANCE", "distance_from_course_m": 8.0, "inside_course_geometry": False},
            {"state": "PARKUR_2_AVOIDANCE", "distance_from_course_m": 2.0, "inside_course_geometry": True},
        ]
        message_id = 1
        for offset, payload in enumerate(states):
            connection.execute("INSERT INTO messages VALUES(?,?,?,?)", (message_id, 1, int((100 + offset) * 1e9), cdr_string(payload)))
            message_id += 1
        for topic_id in (2, 3, 4):
            connection.execute("INSERT INTO messages VALUES(?,?,?,?)", (message_id, topic_id, int(102 * 1e9), cdr_string({"stamp": 102.0})))
            message_id += 1
        connection.commit()
        connection.close()
        return path

    def test_decodes_little_endian_std_string(self):
        self.assertEqual(json.loads(decode_std_string(cdr_string({"x": 1}))), {"x": 1})

    def test_analyzes_maximum_and_continuous_outside_interval(self):
        with tempfile.TemporaryDirectory() as root:
            result = analyze_bag(self.make_bag(root), 5.5, 1.1)["parkur2"]
        self.assertEqual(result["sample_count"], 4)
        self.assertEqual(result["distance_m"]["maximum"], 8.0)
        self.assertEqual(result["distance_m"]["final"], 2.0)
        self.assertEqual(result["outside_intervals_s_from_p2_start"], [[1.0, 2.0]])
        self.assertEqual(result["longest_continuous_outside_s"], 1.0)

    def test_rejects_missing_required_topics(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "empty.db3"
            connection = sqlite3.connect(path)
            connection.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT)")
            connection.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
            connection.close()
            with self.assertRaisesRegex(ValueError, "required topic missing"):
                analyze_bag(path)


if __name__ == "__main__":
    unittest.main()
