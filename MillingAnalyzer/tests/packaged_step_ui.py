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
                browser.close()
        finally:
            app.terminate()
