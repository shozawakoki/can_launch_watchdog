#!/usr/bin/env python3
"""Restart an existing ROS 2 launch when ZLAC8015D CAN feedback stops.

Ubuntu 22.04 / ROS 2 Humble. No third-party Python dependencies.
Example:
    python3 can_launch_watchdog.py
    python3 can_launch_watchdog.py --can-interface can0 --node-id 1
"""

import argparse
import os
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime

CAN_SFF_MASK = 0x7FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_FRAME_SIZE = 16

STOP_REQUESTED = False


def on_signal(signum, _frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


def log(message):
    stamp = datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S')
    print(f'[{stamp}] [CAN-WATCHDOG] {message}', flush=True)


def open_can_socket(interface, can_ids):
    """Listen for exact standard data-frame IDs, without consuming other sockets' frames."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        frame_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
        filters = b''.join(struct.pack('=II', can_id, frame_mask) for can_id in can_ids)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        sock.bind((interface,))
        sock.setblocking(False)
        return sock
    except Exception:
        sock.close()
        raise


def stop_launch(process):
    """Ask ROS launch to shut down all nodes; escalate only when needed."""
    if process is None:
        return

    if process.poll() is not None:
        return

    log('既存launchを停止します (SIGINT)。')
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        log('正常終了しないためSIGTERMを送信します。')
    except ProcessLookupError:
        return

    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        log('終了しないプロセスをSIGKILLします。')
    except ProcessLookupError:
        return

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        log('警告: プロセス終了を確認できませんでした。')


def failure_reason(last_seen, launched_at, now, startup_timeout, feedback_timeout):
    """Wait for both feedback streams on startup, then enforce a staleness timeout."""
    missing = [can_id for can_id, last in last_seen.items() if last is None]
    if missing:
        if now - launched_at >= startup_timeout:
            return '起動後に受信できないCAN ID: ' + ', '.join(f'0x{x:03X}' for x in missing)
        return None

    stale = [can_id for can_id, last in last_seen.items()
             if now - last >= feedback_timeout]
    if stale:
        return (f'{feedback_timeout:g}秒以上受信がないCAN ID: ' +
                ', '.join(f'0x{x:03X}' for x in stale))
    return None


def file_mtime_ns(path):
    try:
        return os.stat(path).st_mtime_ns
    except FileNotFoundError:
        return None


def monitor_one_launch(args):
    """Return the reason for restart, or None if the supervisor is stopping."""
    can_ids = (0x180 + args.node_id, 0x380 + args.node_id)
    try:
        can_sock = open_can_socket(args.can_interface, can_ids)
    except OSError as exc:
        log(f'CANソケットを開けません ({args.can_interface}): {exc}')
        return 'CANソケット接続失敗'

    process = None
    try:
        command = ['ros2', 'launch', args.package, args.launch_file]
        log('起動: ' + ' '.join(command))
        old_log_mtime = file_mtime_ns(args.log_file) if args.log_file else None
        try:
            process = subprocess.Popen(command, start_new_session=True)
        except OSError as exc:
            log(f'launch起動失敗: {exc}')
            return 'launch実行失敗'

        launched_at = time.monotonic()
        last_seen = {can_id: None for can_id in can_ids}
        success_announced = False
        last_status = launched_at
        last_log_mtime = old_log_mtime
        last_log_change = None
        log_announced = False

        while not STOP_REQUESTED:
            exit_code = process.poll()
            if exit_code is not None:
                return f'launchが終了しました (exit={exit_code})'

            try:
                ready, _, _ = select.select([can_sock], [], [], 0.2)
                if ready:
                    packet = can_sock.recv(CAN_FRAME_SIZE)
                    if len(packet) == CAN_FRAME_SIZE:
                        can_id, dlc = struct.unpack_from('=IB', packet)
                        # Only both-wheel feedback frames (8 payload bytes).
                        if can_id in last_seen and dlc >= 8:
                            last_seen[can_id] = time.monotonic()
            except OSError as exc:
                return f'CAN受信エラー: {exc}'

            now = time.monotonic()
            if not success_announced and all(t is not None for t in last_seen.values()):
                success_announced = True
                log('正常受信を確認: ' + ', '.join(f'0x{x:03X}' for x in can_ids))

            reason = failure_reason(
                last_seen, launched_at, now, args.startup_timeout, args.feedback_timeout)
            if reason:
                return reason

            # LADRC hardware flushes this CSV after 200 successful writes
            # (about 1 s at 200 Hz). If the control pipeline stops, raw CAN
            # frames could continue even though ROS has stopped processing.
            if args.log_file:
                current_mtime = file_mtime_ns(args.log_file)
                if current_mtime is not None and current_mtime != last_log_mtime:
                    last_log_mtime = current_mtime
                    last_log_change = now
                    if not log_announced:
                        log('制御ログ更新を確認: ' + args.log_file)
                        log_announced = True
                if last_log_change is None and now - launched_at >= args.startup_timeout:
                    return f'起動後も制御ログが更新されません: {args.log_file}'
                if last_log_change is not None and now - last_log_change >= args.log_timeout:
                    return f'{args.log_timeout:g}秒以上制御ログが更新されません: {args.log_file}'

            if success_announced and now - last_status >= args.status_interval:
                ages = ', '.join(f'0x{x:03X}={now - last_seen[x]:.2f}s' for x in can_ids)
                log('監視中（最終受信からの経過時間）: ' + ages)
                last_status = now
        return None
    finally:
        stop_launch(process)
        can_sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can-interface', default='can0', help='SocketCAN interface')
    parser.add_argument('--node-id', type=int, default=1, help='CANopen node ID, 1-127')
    parser.add_argument('--package', default='zlac_can_bringup', help='ROS 2 package')
    parser.add_argument('--launch-file', default='zlac_can_bringup.launch.py')
    parser.add_argument('--startup-timeout', type=float, default=15.0,
                        help='Maximum seconds to wait for both feedback IDs after launch')
    parser.add_argument('--feedback-timeout', type=float, default=2.0,
                        help='Restart if either feedback stream is silent for this long')
    parser.add_argument('--log-file', default='/tmp/ladrc_track_200hz.csv',
                        help='LADRC runtime CSV path (empty string disables the check)')
    parser.add_argument('--log-timeout', type=float, default=5.0,
                        help='Restart if the LADRC CSV is not updated for this many seconds')
    parser.add_argument('--restart-delay', type=float, default=3.0,
                        help='Seconds between shutdown and relaunch')
    parser.add_argument('--status-interval', type=float, default=10.0,
                        help='Seconds between healthy status log lines')
    args = parser.parse_args()

    if not 1 <= args.node_id <= 127:
        parser.error('--node-id must be 1..127')
    for name in ('startup_timeout', 'feedback_timeout', 'log_timeout', 'restart_delay', 'status_interval'):
        if getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be positive')

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    log(f'監視開始: {args.can_interface}, node={args.node_id}, '
        f'起動猶予={args.startup_timeout:g}s, 無受信判定={args.feedback_timeout:g}s, '
        f'ログ監視={args.log_file or "OFF"}')

    restarts = 0
    while not STOP_REQUESTED:
        reason = monitor_one_launch(args)
        if STOP_REQUESTED:
            break
        restarts += 1
        log(f'異常: {reason} → 再起動 #{restarts} ({args.restart_delay:g}秒後)')
        deadline = time.monotonic() + args.restart_delay
        while not STOP_REQUESTED and time.monotonic() < deadline:
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
    log('監視を終了しました。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
