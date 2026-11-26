"""Drive the packaged native Windows form via its embedded WebView2; no external browser."""
import csv
import io
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

from playwright.sync_api import expect, sync_playwright

package = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory() as tmp, socket.socket() as probe:
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    env = dict(os.environ, WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS=f'--remote-debugging-port={port}',
               WEBVIEW2_USER_DATA_FOLDER=str(Path(tmp)/'profile'))
    app = subprocess.Popen([package/'MillingCAM.exe'], cwd=tmp, env=env)
    try:
        deadline = time.monotonic()+65
        while time.monotonic()<deadline:
            assert app.poll() is None, app.returncode
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/json/version', timeout=1):
                    break
            except OSError:
                time.sleep(.25)
        else: raise RuntimeError('Desktop WebView2 did not start')
        with sync_playwright() as pw:
            browser=pw.chromium.connect_over_cdp(f'http://127.0.0.1:{port}')
            ctx=browser.contexts[0]
            page=ctx.pages[0] if ctx.pages else ctx.wait_for_event('page')
            expect(page.locator('#stockFramePanel')).to_be_visible(timeout=30000)
            expect(page.locator('#egmPanel')).to_be_hidden()
            model=Path(tmp)/'Деревянный брусок.step'
            shutil.copyfile(package/'examples'/'automatic-box.step',model)
            page.locator('#file').set_input_files(model)
            expect(page.locator('#modelInfo')).to_contain_text(model.name,timeout=30000)
            assert page.evaluate('hitTriangles.length>0')
            page.locator('#solidDemo').click()
            expect(page.locator('#saveStockPath')).to_be_enabled(timeout=60000)
            page.locator('#frameMode').select_option('three')
            for field, value in [('frameO','10 20 5'),('frameX','10 30 5'),('frameY','0 20 5')]:
                page.locator('#'+field).fill(value)
            # All coordinates and the local TCP basis must transform together.
            assert page.evaluate('framePoint(stockFrame,[8,23,4])') == [3,2,-1]
            assert page.evaluate('norm(sub(rotate(frameQuaternion(stockFrame,[1,0,0,0]),[1,0,0]),[0,-1,0])) < 1e-9')
            result=page.evaluate("""() => {
                const p=face.trajectory.poses[0], expected=framePoint(stockFrame,p.p);
                const old=p.collision; p.collision=true;
                try { return [workObjectCsv(),expected]; } finally { p.collision=old; }
            }""")
            rows=list(csv.DictReader(io.StringIO(result[0])))
            assert len(rows)>1 and rows[0]['collision']=='1'
            assert [float(rows[0][k]) for k in ('part_x_mm','part_y_mm','part_z_mm')]==result[1]
            page.locator('#frameY').fill('10 40 5')
            expect(page.locator('#saveStockPath')).to_be_disabled()
            page.locator('#frameMode').select_option('stock')
            expect(page.locator('#saveStockPath')).to_be_enabled()
            print('Packaged desktop: STEP, 3D, program export, taught axes and collision flags passed.')
    finally:
        subprocess.run(['taskkill','/PID',str(app.pid),'/T','/F'],capture_output=True)
        app.wait(timeout=10)
