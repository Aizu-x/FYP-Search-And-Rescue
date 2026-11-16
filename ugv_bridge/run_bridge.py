"""Start the UGV bridge on the Raspberry Pi.

    python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend dryrun
    python3 -m ugv_bridge.run_bridge --config ugv_bridge/config/bridge_config.json --backend ros2

The bridge starts DISARMED and never moves the robot by itself. It is not
installed as a service; run it manually in a terminal (Ctrl+C stops it and
publishes zero velocity).
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import stat
import sys
import threading
import time

from .protocol import load_key
from .server import BridgeConfig, BridgeServer


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', required=True)
    parser.add_argument('--backend', choices=['ros2', 'dryrun'], required=True,
                        help='dryrun never touches motors (communication tests); ros2 forwards to the HiWonder controller')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format='%(asctime)s %(levelname)s %(message)s')
    with open(args.config, encoding='utf-8') as handle:
        raw = json.load(handle)
    topics = raw.pop('topics', None)
    raw.pop('backend', None)
    config = BridgeConfig(raw)
    key_path = config.key_file
    if not os.path.isabs(key_path):
        key_path = os.path.join(os.path.dirname(os.path.abspath(args.config)), key_path)
    if os.name == 'posix' and os.stat(key_path).st_mode & (stat.S_IRWXG | stat.S_IRWXO):
        sys.exit('Refusing to start: key file %s is readable by other users. Run: chmod 600 %s' % (key_path, key_path))
    psk = load_key(key_path)
    if args.backend == 'ros2':
        if not topics or not topics.get('cmd_vel'):
            sys.exit('Config must define topics.cmd_vel (verify with tools/discover_mentorpi.sh).')
        from .backends.ros2_backend import Ros2Backend
        backend = Ros2Backend(topics)
    else:
        from .backends.dryrun_backend import DryRunBackend
        backend = DryRunBackend()
    server = BridgeServer(config, backend, psk).start()
    print('UGV bridge listening on %s:%d with backend=%s. State: DISARMED. Ctrl+C to stop.'
          % (config.bind_host, server.port, backend.name), flush=True)
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    try:
        while not done.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        print('Bridge stopped; zero velocity published.')


if __name__ == '__main__':
    main()
