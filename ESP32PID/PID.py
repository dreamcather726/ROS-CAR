import argparse
import queue
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    return [p.device for p in ports if p.device]


def parse_pair(line: str):
    s = line.strip()
    if not s:
        return None
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 2:
        return None
    try:
        a = float(parts[0])
        b = float(parts[1])
    except ValueError:
        return None
    return a, b


def reader_thread(ser: serial.Serial, out_q: "queue.SimpleQueue[tuple[float, float]]", stop_evt: threading.Event):
    buf = bytearray()
    while not stop_evt.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except Exception:
            break
        if not chunk:
            time.sleep(0.002)
            continue
        buf.extend(chunk)
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            raw = bytes(buf[:nl]).rstrip(b"\r")
            del buf[: nl + 1]
            try:
                line = raw.decode("utf-8", errors="ignore")
            except Exception:
                continue
            pair = parse_pair(line)
            if pair is None:
                continue
            out_q.put(pair)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto", help="串口端口，例如 COM6 或 /dev/ttyUSB0；auto 表示自动选择第一个")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=int, default=300, help="显示窗口长度（点数）")
    ap.add_argument(
        "--order",
        choices=["target,actual", "actual,target"],
        default="target,actual",
        help="串口一行两列数据的顺序",
    )
    args = ap.parse_args()

    port = args.port
    if port.lower() == "auto":
        ports = list_ports()
        if not ports:
            raise SystemExit("未找到可用串口，请使用 --port 指定，例如 --port COM6")
        port = ports[0]

    ser = serial.Serial(port=port, baudrate=args.baud, timeout=0.05)
    print(f"Serial open: {port} @ {args.baud}")
    print("Expect line format: target,actual")

    data_q: "queue.SimpleQueue[tuple[float, float]]" = queue.SimpleQueue()
    stop_evt = threading.Event()
    th = threading.Thread(target=reader_thread, args=(ser, data_q, stop_evt), daemon=True)
    th.start()

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except Exception as e:
        stop_evt.set()
        ser.close()
        raise SystemExit(f"需要安装 matplotlib 才能画曲线：{e}")

    xs = deque(maxlen=args.window)
    target = deque(maxlen=args.window)
    actual = deque(maxlen=args.window)

    fig, ax = plt.subplots()
    ln_t, = ax.plot([], [], label="target")
    ln_a, = ax.plot([], [], label="actual")
    ax.set_title("ESP32 PID Monitor")
    ax.set_xlabel("sample")
    ax.set_ylabel("value")
    ax.grid(True)
    ax.legend(loc="upper right")

    sample_idx = 0

    def update(_frame):
        nonlocal sample_idx
        got = 0
        while True:
            try:
                a, b = data_q.get_nowait()
            except Exception:
                break
            if args.order == "target,actual":
                t, y = a, b
            else:
                y, t = a, b
            xs.append(sample_idx)
            target.append(t)
            actual.append(y)
            sample_idx += 1
            got += 1

        if not xs:
            return ln_t, ln_a

        ln_t.set_data(xs, target)
        ln_a.set_data(xs, actual)
        ax.set_xlim(xs[0], xs[-1])

        y_min = min(min(target), min(actual))
        y_max = max(max(target), max(actual))
        if y_min == y_max:
            y_min -= 1.0
            y_max += 1.0
        pad = (y_max - y_min) * 0.1
        ax.set_ylim(y_min - pad, y_max + pad)
        return ln_t, ln_a

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    try:
        plt.show()
    finally:
        stop_evt.set()
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
