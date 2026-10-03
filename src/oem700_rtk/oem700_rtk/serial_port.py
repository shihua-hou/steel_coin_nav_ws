"""115200 8N1 serial open/probe without pyserial."""

import glob
import os
import select
import termios


def candidate_ports():
    ports = []
    if os.path.exists("/dev/oem700_nmea"):
        ports.append("/dev/oem700_nmea")
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        # Air700E 的 if06 是 UM982 透传口，稳定出 NMEA。日志口偶尔夹一条 GGA。
        names = sorted(os.listdir(by_id))
        for name in names:
            if name.endswith("-if06"):
                ports.append(os.path.join(by_id, name))
    # 不要去开另外三个口。RTCM 口一读就几十万字节，反复开关会把整块板子从 USB 上踢掉。
    seen = set()
    out = []
    for path in ports:
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def open_serial(path, baud=115200):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag = 0
        oflag = 0
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
        lflag = 0
        speed = getattr(termios, "B%d" % int(baud))
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(
            fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def write_all(fd, data):
    if isinstance(data, str):
        data = data.encode("ascii", "ignore")
    view = memoryview(data)
    while view:
        n = os.write(fd, view)
        view = view[n:]


def read_some(fd, timeout, limit=4096):
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return b""
    try:
        return os.read(fd, limit)
    except BlockingIOError:
        return b""
