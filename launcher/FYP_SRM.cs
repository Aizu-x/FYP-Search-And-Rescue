// FYP_SRM.exe: opens the console without a terminal window.
//
// It does what "python nav_main.py gui" does in a terminal, and nothing else:
//   - finds the Python environment (FYP_SRM.ini, else .venv in this folder, else python on the PATH),
//   - starts  python -u launcher\run_console.py  hidden, in the project folder; the console's messages are
//     written to launcher\last_run.log (there is no terminal to show them),
//   - shows "Starting..." until the console window is there, and the messages in an error box if it fails,
//   - started again while the console is running, it brings the console window back instead of starting a
//     second console.
// Once started, the console does not depend on this launcher: its messages go straight into the log file.
// The launcher never talks to the robot or the drone; everything the console does is done by the console.
//
// Its picture is launcher\icon.png. BUILD.cmd builds that picture into the .exe (the icon Windows shows for the
// file), and the "Starting..." window reads the same file. For a new design: replace icon.png, run BUILD.cmd.
//
// Build: launcher\BUILD.cmd (uses the C# compiler that is part of Windows; nothing has to be installed).

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;

[assembly: AssemblyTitle("FYP_SRM console")]
[assembly: AssemblyDescription("Opens the UGV / UAV console without a terminal window")]
[assembly: AssemblyProduct("FYP_SRM")]
[assembly: AssemblyVersion("1.0.0.0")]
[assembly: AssemblyFileVersion("1.0.0.0")]

static class Launcher
{
    const string Title = "FYP_SRM console";
    static readonly Regex UrlPattern = new Regex(@"http://127\.0\.0\.1:\d+/#k=[A-Za-z0-9_\-]+");
    static readonly string[] PageTitles = { "UGV Search & Rescue", "UAV \u00B7 Search & Rescue" };

    static string root, launcherDir, logPath, statePath, pythonUsed;
    static string[] arguments;
    static System.Windows.Forms.Timer timer;
    static Splash splash;
    static IntPtr process = IntPtr.Zero;    // the console, when this launcher started it
    static Process adopted;                 // the console, when an earlier launcher started it and it still runs
    static int processId;
    static long processCreated;
    static string url;
    static DateTime started, urlSeen;
    static bool pageOpenedHere, startupDone;
    static int exitCode;

    [STAThread]
    static int Main(string[] args)
    {
        try { SetProcessDPIAware(); } catch (Exception) { }
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        try
        {
            return Run(args);
        }
        catch (Exception error)
        {
            Fail("The launcher could not start the console:\n\n" + error.Message);
            return 3;
        }
    }

    static int Run(string[] args)
    {
        arguments = args;
        root = FindRoot();
        if (root == null)
        {
            Fail("FYP_SRM.exe has to stay in the project folder, next to nav_main.py.\n\n"
                 + "To start it from somewhere else (the Desktop, the taskbar), make a shortcut to it "
                 + "instead of moving or copying it.");
            return 2;
        }
        launcherDir = Path.Combine(root, "launcher");
        logPath = Path.Combine(launcherDir, "last_run.log");
        statePath = Path.Combine(launcherDir, "running.txt");

        string id = "FYP_SRM_" + Hash(root.ToLowerInvariant());
        bool first;
        using (Mutex running = new Mutex(true, id + "_running", out first))
        {
            if (!first)
            {
                // The console is already running: bring its window back and start nothing. (Done here and not
                // by the launcher that is already running: Windows lets the program the user has just started
                // put a window in front.) While the console is still starting there is nothing to show yet.
                string[] known = ReadState();
                if (known != null)
                {
                    url = known[2];
                    ShowConsole();
                }
                return 0;
            }
            if (!Begin())
                return exitCode;
            timer = new System.Windows.Forms.Timer();
            timer.Interval = 200;
            timer.Tick += Tick;
            timer.Start();
            Application.Run();
            GC.KeepAlive(running);
            return exitCode;
        }
    }

    // ------------------------------------------------------------------ starting
    static bool Begin()
    {
        started = DateTime.UtcNow;
        if (Adopt())
        {
            ShowConsole();
            return true;
        }
        string tried;
        string python = FindPython(out tried);
        if (python == null)
        {
            Fail("No Python environment was found for the console.\n\nLooked for:\n" + tried
                 + "\nPut the path of the environment's python.exe in FYP_SRM.ini (next to FYP_SRM.exe), like this:\n\n"
                 + "python=C:\\path\\to\\.venv\\Scripts\\python.exe");
            exitCode = 2;
            return false;
        }
        string shim = Path.Combine(launcherDir, "run_console.py");
        if (!File.Exists(shim))
        {
            Fail("A file of the launcher is missing:\n" + shim);
            exitCode = 2;
            return false;
        }
        pythonUsed = python;
        Start(python, shim);
        if (Environment.GetEnvironmentVariable("FYP_SRM_NO_SPLASH") != "1")     // (set by automated tests)
        {
            splash = new Splash();
            splash.Show();
        }
        return true;
    }

    static string FindRoot()
    {
        string folder = Path.GetDirectoryName(Application.ExecutablePath);
        for (int i = 0; i < 2 && folder != null; i++)       // the project folder, or launcher\ inside it
        {
            if (File.Exists(Path.Combine(folder, "nav_main.py")))
                return folder;
            folder = Path.GetDirectoryName(folder);
        }
        return null;
    }

    static string FindPython(out string tried)
    {
        StringBuilder log = new StringBuilder();
        string ini = Path.Combine(root, "FYP_SRM.ini");
        if (File.Exists(ini))
        {
            foreach (string raw in File.ReadAllLines(ini))
            {
                string line = raw.Trim();
                int cut = line.IndexOf('=');
                if (line.Length == 0 || line[0] == ';' || line[0] == '#' || cut < 0)
                    continue;
                if (!string.Equals(line.Substring(0, cut).Trim(), "python", StringComparison.OrdinalIgnoreCase))
                    continue;
                string value = Environment.ExpandEnvironmentVariables(line.Substring(cut + 1).Trim().Trim('"'));
                if (value.Length == 0)
                    continue;
                string path = Path.IsPathRooted(value) ? value : Path.Combine(root, value);
                log.AppendLine("  " + path + "   (from FYP_SRM.ini)");
                if (File.Exists(path))
                {
                    tried = log.ToString();
                    return path;
                }
            }
        }
        string local = Path.Combine(root, ".venv", "Scripts", "python.exe");
        log.AppendLine("  " + local);
        if (File.Exists(local))
        {
            tried = log.ToString();
            return local;
        }
        foreach (string folder in (Environment.GetEnvironmentVariable("PATH") ?? "").Split(';'))
        {
            string trimmed = folder.Trim().Trim('"');
            if (trimmed.Length == 0 || trimmed.IndexOf("WindowsApps", StringComparison.OrdinalIgnoreCase) >= 0)
                continue;                                   // WindowsApps\python.exe only opens the Store
            try
            {
                string path = Path.Combine(trimmed, "python.exe");
                if (File.Exists(path))
                {
                    tried = log.ToString();
                    return path;
                }
            }
            catch (ArgumentException) { }
        }
        log.AppendLine("  python.exe on the PATH");
        tried = log.ToString();
        return null;
    }

    static void Start(string python, string shim)
    {
        Directory.CreateDirectory(launcherDir);
        try { File.Delete(statePath); } catch (Exception) { }
        SECURITY_ATTRIBUTES inherit = new SECURITY_ATTRIBUTES();
        inherit.nLength = Marshal.SizeOf(typeof(SECURITY_ATTRIBUTES));
        inherit.bInheritHandle = true;
        IntPtr log = CreateFile(logPath, GENERIC_WRITE, FILE_SHARE_ALL, ref inherit, CREATE_ALWAYS,
                                FILE_ATTRIBUTE_NORMAL, IntPtr.Zero);
        if (log == INVALID_HANDLE)
            throw new Win32Exception(Marshal.GetLastWin32Error(), "Cannot write " + logPath);
        IntPtr nothing = CreateFile("NUL", GENERIC_READ, FILE_SHARE_ALL, ref inherit, OPEN_EXISTING, 0, IntPtr.Zero);

        STARTUPINFO info = new STARTUPINFO();
        info.cb = Marshal.SizeOf(typeof(STARTUPINFO));
        info.dwFlags = STARTF_USESTDHANDLES;
        info.hStdInput = nothing;
        info.hStdOutput = log;
        info.hStdError = log;

        StringBuilder command = new StringBuilder(Quote(python) + " -u " + Quote(shim));
        foreach (string argument in arguments)
            command.Append(' ').Append(Quote(argument));

        PROCESS_INFORMATION made;
        bool ok = CreateProcess(null, command, IntPtr.Zero, IntPtr.Zero, true, CREATE_NO_WINDOW, IntPtr.Zero, root,
                                ref info, out made);
        int error = Marshal.GetLastWin32Error();
        CloseHandle(log);
        if (nothing != INVALID_HANDLE)
            CloseHandle(nothing);
        if (!ok)
            throw new Win32Exception(error, "Cannot start " + python + ": " + new Win32Exception(error).Message);
        CloseHandle(made.hThread);
        process = made.hProcess;
        processId = made.dwProcessId;
        long ended, kernel, user;
        if (!GetProcessTimes(process, out processCreated, out ended, out kernel, out user))
            processCreated = 0;
    }

    /// A console started by an earlier launcher that is still running (the launcher was closed, not the console).
    static bool Adopt()
    {
        try
        {
            string[] lines = ReadState();
            if (lines == null)
                return false;
            int id = int.Parse(lines[0]);
            long created = long.Parse(lines[1]);
            Process candidate = Process.GetProcessById(id);
            if (candidate.HasExited
                    || !candidate.ProcessName.StartsWith("python", StringComparison.OrdinalIgnoreCase)
                    || Math.Abs(candidate.StartTime.ToFileTimeUtc() - created) > 100000)
                throw new InvalidOperationException();
            adopted = candidate;
            processId = id;
            processCreated = created;
            url = lines[2];
            urlSeen = DateTime.UtcNow;
            startupDone = true;
            return true;
        }
        catch (Exception)
        {
            try { File.Delete(statePath); } catch (Exception) { }
            return false;
        }
    }

    /// launcher\running.txt, written when the console is ready: its process id, when it was created, its link.
    static string[] ReadState()
    {
        try
        {
            if (!File.Exists(statePath))
                return null;
            string[] lines = File.ReadAllLines(statePath);
            return lines.Length >= 3 && UrlPattern.IsMatch(lines[2]) ? lines : null;
        }
        catch (Exception)
        {
            return null;
        }
    }

    // ------------------------------------------------------------------ while it runs
    static void Tick(object sender, EventArgs e)
    {
        bool exited;
        int code;
        State(out exited, out code);

        if (url == null)
        {
            Match found = UrlPattern.Match(ReadLog(0, 65536));
            if (found.Success)
            {
                url = found.Value;
                urlSeen = DateTime.UtcNow;
                try { File.WriteAllLines(statePath, new string[] { processId.ToString(), processCreated.ToString(), url }); }
                catch (Exception) { }
            }
            else if (splash != null && !exited)
            {
                splash.Waiting((int)(DateTime.UtcNow - started).TotalSeconds);
            }
        }
        if (url != null && !startupDone)
        {
            // the console opens its own window; if it found no browser to do that, open the page here
            if (!pageOpenedHere && Array.IndexOf(arguments, "--no-browser") < 0
                    && ReadLog(0, 65536).Contains("Open the link above"))
            {
                pageOpenedHere = true;
                OpenPage();
            }
            if ((DateTime.UtcNow - urlSeen).TotalSeconds > 2.0)
            {
                CloseSplash();
                startupDone = true;
            }
        }
        if (exited)
        {
            timer.Stop();
            CloseSplash();
            try { File.Delete(statePath); } catch (Exception) { }
            if (code != 0 || url == null)
            {
                exitCode = 1;
                string all = ReadLog(-1, 6000).Trim();
                string tail = LastLines(all, 12);
                string hint = all.Contains("ModuleNotFoundError")
                    ? "\n\nThis Python environment is missing packages. Install them with:\n"
                      + Quote(pythonUsed ?? "python") + " -m pip install -r requirements.txt -r requirements-yolo.txt"
                    : "";
                Fail("The console stopped" + (code != 0 ? " with an error (code " + code + ")" : " before it was ready")
                     + ".\n\nLast messages:\n" + (tail.Length > 0 ? tail : "(none)") + hint + "\n\nFull log: " + logPath);
            }
            Application.ExitThread();
        }
    }

    static void State(out bool exited, out int code)
    {
        code = 0;
        if (process != IntPtr.Zero)
        {
            exited = WaitForSingleObject(process, 0) == 0;
            uint value;
            if (exited && GetExitCodeProcess(process, out value))
                code = unchecked((int)value);
        }
        else
        {
            try { exited = adopted.HasExited; }
            catch (Exception) { exited = true; }
        }
    }

    static void CloseSplash()
    {
        if (splash != null)
        {
            splash.Close();
            splash.Dispose();
            splash = null;
        }
    }

    /// Bring the console's window to the front; open the page again when its window was closed.
    static void ShowConsole()
    {
        IntPtr window = IntPtr.Zero;
        EnumWindows(delegate(IntPtr handle, IntPtr unused)
        {
            if (!IsWindowVisible(handle))
                return true;
            StringBuilder text = new StringBuilder(256);
            GetWindowText(handle, text, text.Capacity);
            string title = text.ToString();
            foreach (string page in PageTitles)
            {
                if (title.StartsWith(page, StringComparison.Ordinal))
                {
                    window = handle;
                    return false;
                }
            }
            return true;
        }, IntPtr.Zero);
        if (window != IntPtr.Zero)
        {
            if (IsIconic(window))
                ShowWindow(window, SW_RESTORE);
            SetForegroundWindow(window);
        }
        else
        {
            OpenPage();
        }
    }

    /// The same way the console opens its page: an app-style Edge / Chrome window, else the default browser.
    static void OpenPage()
    {
        string[] folders = { Environment.GetEnvironmentVariable("PROGRAMFILES(X86)"),
                             Environment.GetEnvironmentVariable("PROGRAMFILES"),
                             Environment.GetEnvironmentVariable("LOCALAPPDATA") };
        string[] browsers = { @"Microsoft\Edge\Application\msedge.exe", @"Google\Chrome\Application\chrome.exe",
                              @"BraveSoftware\Brave-Browser\Application\brave.exe" };
        foreach (string folder in folders)
        {
            if (string.IsNullOrEmpty(folder))
                continue;
            foreach (string browser in browsers)
            {
                string exe = Path.Combine(folder, browser);
                if (!File.Exists(exe))
                    continue;
                try
                {
                    ProcessStartInfo open = new ProcessStartInfo(exe, "--app=" + url + " --window-size=1520,940");
                    open.UseShellExecute = false;
                    Process.Start(open);
                    return;
                }
                catch (Exception) { }
            }
        }
        try { Process.Start(url); } catch (Exception) { }
    }

    // ------------------------------------------------------------------ small helpers
    static string ReadLog(int from, int count)
    {
        try
        {
            using (FileStream stream = new FileStream(logPath, FileMode.Open, FileAccess.Read,
                                                      FileShare.ReadWrite | FileShare.Delete))
            {
                if (from < 0)
                    stream.Seek(Math.Max(0, stream.Length - count), SeekOrigin.Begin);
                byte[] buffer = new byte[count];
                int read = stream.Read(buffer, 0, count);
                return Encoding.Default.GetString(buffer, 0, read);
            }
        }
        catch (Exception)
        {
            return "";
        }
    }

    static string LastLines(string text, int count)
    {
        string[] lines = text.Replace("\r", "").Split('\n');
        int from = Math.Max(0, lines.Length - count);
        return (from > 0 ? "...\n" : "") + string.Join("\n", lines, from, lines.Length - from);
    }

    static void Fail(string message)
    {
        if (timer != null)
            timer.Stop();
        // 0x40000 = MB_TOPMOST: the box must not open behind other windows, nobody would see it there
        MessageBox.Show(message, Title, MessageBoxButtons.OK, MessageBoxIcon.Error, MessageBoxDefaultButton.Button1,
                        (MessageBoxOptions)0x40000);
    }

    static string Hash(string text)
    {
        using (SHA256 sha = SHA256.Create())
            return BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(text)), 0, 8).Replace("-", "");
    }

    /// One command-line argument, quoted the way the Windows C runtime (and so python.exe) reads it back.
    static string Quote(string argument)
    {
        if (argument.Length > 0 && argument.IndexOfAny(new char[] { ' ', '\t', '"' }) < 0)
            return argument;
        StringBuilder quoted = new StringBuilder("\"");
        int slashes = 0;
        foreach (char c in argument)
        {
            if (c == '\\')
            {
                slashes++;
                continue;
            }
            if (c == '"')
            {
                quoted.Append('\\', slashes * 2 + 1);
                quoted.Append('"');
            }
            else
            {
                quoted.Append('\\', slashes);
                quoted.Append(c);
            }
            slashes = 0;
        }
        quoted.Append('\\', slashes * 2);
        quoted.Append('"');
        return quoted.ToString();
    }

    // ------------------------------------------------------------------ the "Starting..." window
    sealed class Splash : Form
    {
        readonly Label line = new Label();

        public Splash()
        {
            float scale;
            using (Graphics g = CreateGraphics())
                scale = g.DpiX / 96f;
            Text = Title;
            FormBorderStyle = FormBorderStyle.None;
            StartPosition = FormStartPosition.CenterScreen;
            ShowInTaskbar = false;
            BackColor = Color.FromArgb(17, 24, 39);
            ClientSize = new Size((int)(430 * scale), (int)(112 * scale));
            int side = (int)(56 * scale);
            Bitmap image = Picture(side);
            if (image != null)
            {
                PictureBox picture = new PictureBox();
                picture.Image = image;
                picture.SetBounds((int)(24 * scale), (int)(28 * scale), side, side);
                Controls.Add(picture);
            }
            Label name = new Label();
            name.Text = Title;
            name.ForeColor = Color.White;
            name.Font = new Font("Segoe UI", 14f, FontStyle.Bold);
            name.AutoSize = true;
            name.Location = new Point((int)(100 * scale), (int)(28 * scale));
            Controls.Add(name);
            line.Text = "Starting...";
            line.ForeColor = Color.FromArgb(156, 163, 175);
            line.Font = new Font("Segoe UI", 9.5f);
            line.AutoSize = true;
            line.Location = new Point((int)(102 * scale), (int)(62 * scale));
            Controls.Add(line);
        }

        // On top of other windows, but without taking the keyboard away from what the user is doing.
        protected override bool ShowWithoutActivation
        {
            get { return true; }
        }

        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams settings = base.CreateParams;
                settings.ExStyle |= 0x00000008 | 0x08000000;        // WS_EX_TOPMOST, WS_EX_NOACTIVATE
                return settings;
            }
        }

        /// The launcher's picture: launcher\icon.png (the file to replace for a new design), drawn at this size.
        /// When that file is missing or unreadable: the icon that was built into this program.
        static Bitmap Picture(int side)
        {
            Image source = null;
            try
            {
                // read through memory, so the file itself is not kept open and can be replaced at any time
                byte[] data = File.ReadAllBytes(Path.Combine(launcherDir, "icon.png"));
                source = Image.FromStream(new MemoryStream(data));
            }
            catch (Exception)
            {
                try { source = Icon.ExtractAssociatedIcon(Application.ExecutablePath).ToBitmap(); }
                catch (Exception) { return null; }
            }
            try
            {
                Bitmap result = new Bitmap(side, side, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
                double factor = Math.Min((double)side / source.Width, (double)side / source.Height);
                int width = Math.Max(1, (int)Math.Round(source.Width * factor));
                int height = Math.Max(1, (int)Math.Round(source.Height * factor));
                using (Graphics g = Graphics.FromImage(result))
                using (System.Drawing.Imaging.ImageAttributes edges = new System.Drawing.Imaging.ImageAttributes())
                {
                    g.Clear(Color.Transparent);
                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                    g.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.HighQuality;
                    edges.SetWrapMode(System.Drawing.Drawing2D.WrapMode.TileFlipXY);
                    g.DrawImage(source, new Rectangle((side - width) / 2, (side - height) / 2, width, height),
                                0, 0, source.Width, source.Height, GraphicsUnit.Pixel, edges);
                }
                return result;
            }
            catch (Exception)
            {
                return null;
            }
            finally
            {
                source.Dispose();
            }
        }

        public void Waiting(int seconds)
        {
            string text = seconds < 20 ? "Starting..." : "Still starting... (" + seconds + " s)";
            if (line.Text != text)
                line.Text = text;
        }
    }

    // ------------------------------------------------------------------ Windows calls
    const uint GENERIC_READ = 0x80000000, GENERIC_WRITE = 0x40000000, FILE_SHARE_ALL = 7;
    const uint CREATE_ALWAYS = 2, OPEN_EXISTING = 3, FILE_ATTRIBUTE_NORMAL = 0x80;
    const uint CREATE_NO_WINDOW = 0x08000000;
    const int STARTF_USESTDHANDLES = 0x100, SW_RESTORE = 9;
    static readonly IntPtr INVALID_HANDLE = new IntPtr(-1);

    [StructLayout(LayoutKind.Sequential)]
    struct SECURITY_ATTRIBUTES
    {
        public int nLength;
        public IntPtr lpSecurityDescriptor;
        [MarshalAs(UnmanagedType.Bool)] public bool bInheritHandle;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    struct STARTUPINFO
    {
        public int cb;
        public string lpReserved, lpDesktop, lpTitle;
        public int dwX, dwY, dwXSize, dwYSize, dwXCountChars, dwYCountChars, dwFillAttribute, dwFlags;
        public short wShowWindow, cbReserved2;
        public IntPtr lpReserved2, hStdInput, hStdOutput, hStdError;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct PROCESS_INFORMATION
    {
        public IntPtr hProcess, hThread;
        public int dwProcessId, dwThreadId;
    }

    delegate bool WindowVisitor(IntPtr handle, IntPtr parameter);

    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    static extern IntPtr CreateFile(string name, uint access, uint share, ref SECURITY_ATTRIBUTES security,
                                    uint disposition, uint flags, IntPtr template);

    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool CreateProcess(string application, StringBuilder command, IntPtr processSecurity,
                                     IntPtr threadSecurity, [MarshalAs(UnmanagedType.Bool)] bool inheritHandles,
                                     uint flags, IntPtr environment, string directory, ref STARTUPINFO info,
                                     out PROCESS_INFORMATION made);

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool CloseHandle(IntPtr handle);

    [DllImport("kernel32.dll", SetLastError = true)]
    static extern uint WaitForSingleObject(IntPtr handle, uint milliseconds);

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool GetExitCodeProcess(IntPtr handle, out uint code);

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool GetProcessTimes(IntPtr handle, out long created, out long ended, out long kernel, out long user);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool SetProcessDPIAware();

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool EnumWindows(WindowVisitor visitor, IntPtr parameter);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    static extern int GetWindowText(IntPtr window, StringBuilder text, int capacity);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool IsWindowVisible(IntPtr window);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool IsIconic(IntPtr window);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool ShowWindow(IntPtr window, int command);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool SetForegroundWindow(IntPtr window);
}
