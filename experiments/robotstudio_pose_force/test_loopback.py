"""Standard-library UDP fake controller: never contacts ABB."""
import csv
import math
import pathlib
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time

EXE = sys.argv[1]
HEADER = 'face_id,pass_id,sample_id,part_x_mm,part_y_mm,part_z_mm,qw,qx,qy,qz,nx,ny,nz,time_s,collision,sweep_collision,motion'
def varint(n):
    result = bytearray()
    while n > 127:
        result.append((n & 127) | 128)
        n >>= 7
    result.append(n)
    return bytes(result)
def integer(key, n): return varint(key * 8) + varint(n)
def message(key, b): return varint(key * 8 + 2) + varint(len(b)) + b
def real(key, x): return varint(key * 8 + 1) + struct.pack('<d', x)
def decode(b):
    p = 0
    def read():
        nonlocal p
        n, shift = 0, 0
        while True:
            v = b[p]; p += 1; n |= (v & 127) << shift
            if v < 128: return n
            shift += 7
    fields = {}
    while p < len(b):
        key = read(); tag, wire = key >> 3, key & 7
        if wire == 0: fields[tag] = read()
        elif wire == 1: fields[tag] = struct.unpack('<d', b[p:p+8])[0]; p += 8
        elif wire == 2:
            n = read(); fields[tag] = b[p:p+n]; p += n
        else: raise AssertionError(wire)
    return fields

def trial(path, stale=False):
    # Constant feedback deliberately anchored at nonzero XYZ and 180 degree X.
    p = b''.join(real(i+1, v) for i, v in enumerate([750, 0, 600]))
    q = b''.join(real(i+1, v) for i, v in enumerate([0, 1, 0, 0]))
    cart = message(1, p) + message(2, q)
    outputs, done, force_id = [], False, 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as robot, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as force:
        robot.bind(('127.0.0.1', 0)); force.bind(('127.0.0.1', 0))
        proc = subprocess.Popen([EXE, '--virtual-only', '--anchor-first-to-feedback', str(path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            assert 'ready' in proc.stdout.readline()
            start = time.monotonic(); next_force = start
            seq = 0
            while proc.poll() is None and time.monotonic() - start < 8:
                now = time.monotonic()
                if now >= next_force and (not stale or now-start < .2):
                    force_id += 1
                    force.sendto(f'{force_id},0.1'.encode(), ('127.0.0.1', 6513))
                    next_force = now + .02
                seq += 1
                header = integer(1, seq) + integer(2, seq*4) + integer(3, 2)
                robot.sendto(message(1, header) + message(2, message(2, cart)), ('127.0.0.1', 6512))
                readable, _, _ = select.select([robot, force], [], [], .004)
                for sock in readable:
                    data, _ = sock.recvfrom(4096)
                    if sock is force:
                        done |= data.startswith(b'DONE,')
                    else:
                        root = decode(data); hdr = decode(root[1]); pose = decode(decode(root[2])[2])
                        xyz, quat = decode(pose[1]), decode(pose[2])
                        assert set(xyz) == {1,2,3} and set(quat) == {1,2,3,4}, 'Explicit zero fields required'
                        assert 3 not in pose, 'Euler must not override quaternion'
                        assert hdr[3] == 3
                        assert abs(sum(v*v for v in quat.values())-1) < 1e-9
                        if outputs: assert hdr[1] > outputs[-1][0]
                        outputs.append((hdr[1], xyz, quat))
                time.sleep(.001)
            stdout, stderr = proc.communicate(timeout=5)
            assert outputs, (stdout, stderr)
            first = outputs[0]
            assert max(abs(first[1][i+1]-v) for i,v in enumerate([750,0,600])) < .002
            if stale:
                assert proc.returncode == 1 and 'Force task stale' in stderr, stderr
            else:
                assert proc.returncode == 0 and done, (stdout, stderr)
                assert outputs[-1][1][1] > 750.9, 'Nominal XYZ trajectory advanced'
                assert abs(outputs[-1][2][3]) > .001, 'Quaternion trajectory changed'
                assert max(abs(v[1][3]-600) for v in outputs) <= .101, 'Bounded normal correction'
                assert min(v[1][3] for v in outputs) >= 599.999, 'Planner outward normal maps to upward +Z for virtual pencil'
                assert abs(first[2][2]) > .999, 'Virtual TCP retains home orientation after frame registration'
        finally:
            if proc.poll() is None: proc.kill(); proc.wait()

with tempfile.TemporaryDirectory() as tmp:
    path = pathlib.Path(tmp)/'whole.poses.csv'
    with path.open('w', newline='') as f:
        f.write(HEADER+'\n')
        writer = csv.writer(f)
        for k in range(6):
            a = .02*k/5
            writer.writerow([1,0,k,k/5,0,0,math.cos(a/2),0,0,math.sin(a/2),0,0,1,k/25,0,0,0])
    trial(path)
    trial(path, stale=True)
    bad = pathlib.Path(tmp)/'bad.csv'
    bad.write_text(path.read_text().replace(',0,0,0\n', ',1,0,0\n'))
    rejected = subprocess.run([EXE,'--virtual-only','--anchor-first-to-feedback',str(bad)], capture_output=True, text=True)
    assert rejected.returncode == 1 and 'Collision flagged' in rejected.stderr
print('Virtual UDP pose/orientation, force limits, completion handshake, stale-force stop and rejection: passed')
