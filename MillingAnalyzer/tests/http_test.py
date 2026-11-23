"""Local API integration: synthetic STEP only, no browser or robot connection."""
import json
import pathlib
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

exe, fixture_exe = sys.argv[1:]
with tempfile.TemporaryDirectory() as tmp:
    path = pathlib.Path(tmp) / 'test.step'
    subprocess.run([fixture_exe, '--fixture', str(path)], check=True, stdout=subprocess.DEVNULL)
    with tempfile.TemporaryFile(mode='w+') as errors:
        proc = subprocess.Popen([exe, '--serve', '--no-open'], stdout=subprocess.PIPE,
                                stderr=errors, text=True)
        try:
            line = proc.stdout.readline()
            assert line.startswith('Milling Analyzer local UI: '), line
            root = line.strip().split(' ')[-1].rstrip('/')
            def request(route, body=b'', status=200, origin=None):
                headers = {'Content-Type': 'application/octet-stream'}
                if origin:
                    headers['Origin'] = origin
                req = urllib.request.Request(root + route, data=body, headers=headers)
                try:
                    with urllib.request.urlopen(req, timeout=60) as r:
                        assert r.status == status
                        return json.load(r)
                except urllib.error.HTTPError as e:
                    assert e.code == status, (e.code, e.read())
                    return json.load(e)
            with urllib.request.urlopen(root + '/') as r:
                page = r.read().decode()
                assert '@@SCENE@@' not in page and '@@API@@' not in page
                assert 'type="file"' in page
            request('/calculate', status=403, origin='https://example.com')
            mesh = request('/load?format=step', path.read_bytes())
            assert len(mesh['faces']) == 6 and all(not f['passes'] for f in mesh['faces'])
            first = request('/calculate?face=1')
            assert first['faces'][0]['passes']
            request('/load?format=step', b'invalid STEP', status=400)
            request('/calculate?face=1&step=nan', status=400)
            request('/calculate?face=1&diameter=-1', status=400)
            request('/calculate?face=1&diameter=1e-300', status=400)
            request('/calculate?face=1.5', status=400)
            again = request('/calculate?face=1')
            assert first == again, 'Failed import must preserve model and poses'
            demo = request('/demo')
            assert demo['faces'][0]['type'] == 'Bezier'
            assert demo['faces'][0]['passes']
            trajectory = demo['faces'][0]['trajectory']['poses']
            assert trajectory and all(p['motion'] != 2 for p in trajectory[1:])
            assert not any(p['collision'] or p['sweepCollision'] for p in trajectory)
            assert all(b['t'] > a['t'] for a, b in zip(trajectory, trajectory[1:]))
            print('HTTP import, selection, calculation, rejection and model preservation: passed')
        finally:
            proc.terminate()
            proc.wait(timeout=10)
