"""Started by FYP_SRM.exe: runs the console exactly like ``python nav_main.py gui`` in a terminal.

The only difference is where the console's messages go. There is no terminal window, so FYP_SRM.exe points
them at ``launcher/last_run.log``. A log file cannot hold every character a terminal can show, so a message
with such a character is written with a replacement mark instead of stopping the console.
"""
import os
import runpy
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors='replace', line_buffering=True)
        except (AttributeError, ValueError):
            pass                                    # no such stream, or not a text stream: leave it alone
    os.chdir(ROOT)
    sys.path[0] = ROOT                              # as if nav_main.py itself had been started
    script = os.path.join(ROOT, 'nav_main.py')
    sys.argv = [script, 'gui'] + sys.argv[1:]
    runpy.run_path(script, run_name='__main__')


if __name__ == '__main__':
    main()
