#!/usr/bin/env python3
'''
listen-before-talk for half-duplex serial links (e.g. RS485 with no bus master)

Before each write on a serial link we wait until no bytes have been
received for a guard interval, so our packets don't start while the
other end (e.g. the autopilot) is mid-transmission. This reduces, but
cannot eliminate, collisions: the other end does not listen before it
talks.

Settings (all live, can be changed with "set" at runtime):
  rs485             enable the guard
  rs485_idle_bytes  guard interval in byte-times at the port baudrate
  rs485_idle_ms     minimum guard interval in ms (covers USB-serial latency)
  rs485_max_wait_ms give up waiting and send anyway after this long
  rs485_jitter_ms   random extra quiet time (0..N ms) required before sending
'''

import random
import threading
import time


def _now():
    return time.monotonic()


def install(conn, settings):
    '''wrap conn.write/conn.recv of a pymavlink mavserial connection'''
    if getattr(conn, 'rs485_installed', False):
        return
    conn.rs485_installed = True
    conn.rs485_last_rx = 0.0
    conn.rs485_waits = 0
    conn.rs485_timeouts = 0
    lock = threading.Lock()
    orig_write = conn.write
    orig_recv = conn.recv

    # ask the OS to deliver bytes promptly (Linux: sets FTDI latency timer to 1ms)
    try:
        conn.port.set_low_latency_mode(True)
    except Exception:
        pass

    def recv(n=None):
        ret = orig_recv(n)
        if ret:
            conn.rs485_last_rx = _now()
        return ret

    def wait_idle():
        port = conn.port
        try:
            baud = float(port.baudrate) or 57600.0
        except Exception:
            baud = 57600.0
        byte_time = 10.0 / baud  # 8N1
        guard = max(settings.rs485_idle_bytes * byte_time,
                    settings.rs485_idle_ms * 0.001)
        if settings.rs485_jitter_ms > 0:
            guard += random.uniform(0, settings.rs485_jitter_ms * 0.001)
        start = _now()
        deadline = start + settings.rs485_max_wait_ms * 0.001
        poll = min(max(byte_time, 0.0002), 0.001)
        # we must actually listen for the whole guard interval, as bytes
        # may be on the wire (or in USB buffers) that we haven't seen yet
        quiet_since = max(conn.rs485_last_rx, start)
        last_waiting = None
        waited = False
        while True:
            now = _now()
            try:
                waiting = port.in_waiting
            except Exception:
                waiting = 0
            if waiting != (0 if last_waiting is None else last_waiting):
                # bytes are still arriving
                quiet_since = now
                conn.rs485_last_rx = now
            last_waiting = waiting
            if now - quiet_since >= guard:
                break
            if now >= deadline:
                conn.rs485_timeouts += 1
                break
            waited = True
            time.sleep(poll)
        if waited:
            conn.rs485_waits += 1

    def write(buf):
        if not settings.rs485:
            return orig_write(buf)
        with lock:
            wait_idle()
            return orig_write(buf)

    conn.write = write
    conn.recv = recv
