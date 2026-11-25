"""Launch the packaged Windows application and import a STEP through Edge."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from playwright.sync_api import expect, sync_playwright

package = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory() as temp:
    model = Path(temp) / 'Деталь с пробелами.STP'
    shutil.copyfile(package / 'examples' / 'automatic-box.step', model)
    with subprocess.Popen([package / 'MillingAnalyzer.exe', '--serve', '--no-open'],
                          cwd=temp, stdout=subprocess.PIPE, text=True) as app:
        try:
            url = app.stdout.readline().split()[-1]
            with sync_playwright() as driver:
                browser = driver.chromium.launch(channel='msedge')
                page = browser.new_page()
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(url)
                # Reproduce importing while the previous view isolates one face.
                page.locator('#all').uncheck()
                with page.expect_file_chooser() as chooser:
                    page.locator('#chooseFile').click()
                chooser.value.set_files(model)
                expect(page.locator('#modelInfo')).to_contain_text(model.name, timeout=30000)
                expect(page.locator('#all')).to_be_checked()
                expect(page.locator('#counts')).to_contain_text('6 граней')
                assert page.evaluate('hitTriangles.length === scene.mesh.length && hitTriangles.length > 0')
                assert page.evaluate('stockOrigin()') == [0, 0, 5]
                assert not errors, errors
                # Open the same Unicode path via the native startup path as well.
                with subprocess.Popen([package / 'MillingAnalyzer.exe','--serve','--no-open','--step',model],
                                      cwd=temp,stdout=subprocess.PIPE,text=True) as direct:
                    try:
                        page.goto(direct.stdout.readline().split()[-1])
                        expect(page.locator('#counts')).to_contain_text('6 граней')
                        assert page.evaluate('hitTriangles.length === scene.mesh.length && hitTriangles.length > 0')
                        expect(page.locator('#egmWorkObject')).to_be_checked()
                    finally:
                        direct.terminate()
                profile=Path(temp)/'Ethernet-IP.txt'
                profile.write_text('192.0.2.10\n192.0.2.20\n')
                with subprocess.Popen([package / 'MillingAnalyzer.exe','--serve','--no-open','--ethernet-profile',profile],
                                      cwd=temp,stdout=subprocess.PIPE,text=True) as ethernet:
                    try:
                        page.goto(ethernet.stdout.readline().split()[-1])
                        expect(page.locator('#egmBind')).to_have_value('192.0.2.10')
                        expect(page.locator('#egmPeer')).to_have_value('192.0.2.20')
                        expect(page.locator('#egmPort')).to_have_value('6510')
                        expect(page.locator('#egmBind')).to_be_disabled()
                        expect(page.locator('#egmWorkObject')).not_to_be_checked()
                        expect(page.locator('#egmCalibrated')).not_to_be_checked()
                    finally:
                        ethernet.terminate()
                browser.close()
        finally:
            app.terminate()
