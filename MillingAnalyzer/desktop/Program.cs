using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;

namespace Sirius.Milling
{
    internal sealed class CamWindow : Form
    {
        private readonly WebView2 viewer = new WebView2 { Dock = DockStyle.Fill };
        private readonly Label progress = new Label {
            Text = "Загрузка CAM…", AutoSize = true, Location = new Point(20, 20)
        };
        private Process engine;
        private bool closing;

        public CamWindow()
        {
            Text = "Milling CAM";
            WindowState = FormWindowState.Maximized;
            MinimumSize = new Size(800, 600);
            Controls.Add(viewer);
            Controls.Add(progress);
            progress.BringToFront();
            Shown += async (_, __) => await Launch();
        }

        private async Task Launch()
        {
            try
            {
                string folder = AppDomain.CurrentDomain.BaseDirectory;
                engine = new Process { StartInfo = new ProcessStartInfo {
                    FileName = Path.Combine(folder, "engine", "MillingAnalyzer.exe"),
                    Arguments = "--desktop-service", WorkingDirectory = Path.Combine(folder, "engine"),
                    UseShellExecute = false, CreateNoWindow = true,
                    RedirectStandardOutput = true, RedirectStandardError = true
                }};
                engine.Start();
                var errors = engine.StandardError.ReadToEndAsync();
                var line = engine.StandardOutput.ReadLineAsync();
                if (await Task.WhenAny(line, Task.Delay(60000)) != line)
                    throw new IOException("Превышено время запуска расчётного модуля.");
                if (closing) return;
                const string prefix = "Milling Analyzer local UI: ";
                string address = await line;
                if (address == null || !address.StartsWith(prefix, StringComparison.Ordinal) ||
                    !Uri.TryCreate(address.Substring(prefix.Length), UriKind.Absolute, out var url) ||
                    url.Scheme != "http" || url.Host != "127.0.0.1")
                    throw new IOException(engine.HasExited ? await errors : "Не получен адрес расчётного модуля.");
                engine.EnableRaisingEvents = true;
                engine.Exited += (_, __) => {
                    if (IsHandleCreated && !closing) BeginInvoke(new Action(() => {
                        if (!closing) { MessageBox.Show(this, "Расчётный модуль завершился.", Text); Close(); }
                    }));
                };
                string userFolder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                                                  "Sirius", "MillingCAM", "WebView2");
                var env = await CoreWebView2Environment.CreateAsync(null, userFolder);
                if (closing) return;
                await viewer.EnsureCoreWebView2Async(env);
                if (closing) return;
                var core = viewer.CoreWebView2;
                core.Settings.AreDefaultContextMenusEnabled = false;
                core.NewWindowRequested += (_, e) => e.Handled = true;
                core.NavigationStarting += (_, e) => {
                    if (e.Uri != url.AbsoluteUri) e.Cancel = true;
                };
                core.DownloadStarting += (_, e) => {
                    e.Handled = true;
                    var deferral = e.GetDeferral();
                    BeginInvoke(new Action(() => {
                        try {
                            using (var dialog = new SaveFileDialog {
                                FileName = Path.GetFileName(e.ResultFilePath), OverwritePrompt = true,
                                Title = "Сохранить результат CAM"
                            }) {
                                if (dialog.ShowDialog(this) == DialogResult.OK) e.ResultFilePath = dialog.FileName;
                                else e.Cancel = true;
                            }
                        } finally { deferral.Complete(); }
                    }));
                };
                core.NavigationCompleted += (_, e) => {
                    if (e.IsSuccess) progress.Visible = false;
                    else progress.Text = "Не удалось открыть модель: " + e.WebErrorStatus;
                };
                core.Navigate(url.AbsoluteUri);
            }
            catch (Exception error)
            {
                if (!closing) {
                    MessageBox.Show(this, error.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
                    Close();
                }
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            closing = true;
            if (engine != null) {
                try { if (!engine.HasExited) engine.Kill(); }
                catch (InvalidOperationException) { }
                engine.Dispose();
            }
            viewer.Dispose();
            base.OnFormClosing(e);
        }

        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new CamWindow());
        }
    }
}
