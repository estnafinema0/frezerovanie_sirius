"""Exercise the single-task mode with a localhost peer that echoes commanded poses."""
import csv
import math
from pathlib import Path
import socket
import subprocess
import sys
import time
from test_loopback import decode, integer, message, real

def pose_bytes(xyz, quat):
    def vector(values):
        return b''.join(real(i, v) for i, v in enumerate(values, 1))
    return message(1, vector(xyz)) + message(2, vector(quat))

def run(exe, path, interrupted=False):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    duration = float(rows[-1]['time_s'])
    pose = pose_bytes((750, 0, 600), (0, 1, 0, 0))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        peer.bind(('127.0.0.1', 0))
        peer.settimeout(.5)
        proc = subprocess.Popen([exe, '--pose-only-6510', '--anchor-first-to-feedback', str(path)],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            greeting = proc.stdout.readline()
            assert 'ready on 127.0.0.1:6510' in greeting, greeting
            start = time.monotonic()
            seq = 0
            while time.monotonic()-start < (.15 if interrupted else duration+.45):
                header = integer(1, seq) + integer(2, seq*4) + integer(3, 2)
                feedback = message(1, header) + message(2, message(2, pose))
                peer.sendto(feedback, ('127.0.0.1', 6510))
                data, addr = peer.recvfrom(4096)
                assert addr[1] == 6510
                root = decode(data)
                assert decode(root[1])[3] == 3
                pose = decode(root[2])[2]
                nested = decode(pose)
                xyz, quat = decode(nested[1]), decode(nested[2])
                assert set(nested) == {1,2} and len(xyz)==3 and len(quat)==4
                if seq == 0:
                    assert list(xyz.values()) == [750,0,600]
                    assert abs(quat[2]) > .99999
                seq += 1
                time.sleep(.004)
            out = proc.communicate(timeout=3)[0]
            if interrupted:
                assert proc.returncode == 1 and 'feedback lost' in out, out
            else:
                assert proc.returncode == 0, out
                if path.name == 'smoke.poses.csv':
                    assert abs(xyz[1]-752)<1e-6 and abs(xyz[2]-2)<1e-6 and abs(xyz[3]-600)<1e-6
                    assert abs(abs(quat[4])-math.sin(math.radians(2)))<1e-7
            print(path.name, seq, 'feedback packets', 'interrupted' if interrupted else 'completed')
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()

if __name__ == '__main__':
    example = Path(__file__).with_name('smoke.poses.csv')
    run(sys.argv[1], example)
    run(sys.argv[1], example, interrupted=True)
    for extra in sys.argv[2:]:
        run(sys.argv[1], Path(extra))
