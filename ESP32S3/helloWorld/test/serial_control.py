import struct
import sys
import tkinter as tk
from tkinter import messagebox, ttk
import threading
import time
import math

HEADER = 0xAA
TAIL = 0xBB

FUNC_ENCODER_COUNTS = 0x01
FUNC_IMU_BUNDLE = 0x02
FUNC_WHEEL_SPEED = 0x10
FUNC_WHEEL_TARGET_COUNTS = 0x11

MAX_PAYLOAD_SIZE = 32

WHEEL_DIAMETER_CM = 7.0
WHEEL_COUNTS_PER_REV = 600.0
WHEEL_CM_PER_COUNT = (WHEEL_DIAMETER_CM * math.pi) / WHEEL_COUNTS_PER_REV


def _crc16(b: bytes) -> int:
    crc = 0xFFFF
    for one_byte in b:
        crc ^= one_byte
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc & 0xFFFF


def _build_frame(func: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError("payload too long")
    data = bytes([func & 0xFF]) + payload
    length = len(data)
    crc_input = bytes([length]) + data
    crc = _crc16(crc_input)
    return bytes([HEADER, length]) + data + bytes([crc & 0xFF, (crc >> 8) & 0xFF, TAIL])


# ====================== 这里修复了！======================
def build_wheel_speed_frame(left_cm_s: float, right_cm_s: float) -> bytes:
    l = int(round(left_cm_s * 100.0))
    r = int(round(right_cm_s * 100.0))
    l = max(-32768, min(32767, l))
    r = max(-32768, min(32767, r))
    # 去掉多余的 + b"\x00\x00"
    payload = struct.pack("<hh", l, r)
    return _build_frame(FUNC_WHEEL_SPEED, payload)
# ========================================================


def _i24_le(v: int) -> bytes:
    v = max(-8388608, min(8388607, int(v)))
    if v < 0:
        v = (1 << 24) + v
    return bytes([(v >> 0) & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def build_wheel_target_counts_frame(left_counts: int, right_counts: int) -> bytes:
    payload = _i24_le(left_counts) + _i24_le(right_counts)
    return _build_frame(FUNC_WHEEL_TARGET_COUNTS, payload)


def _hex(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def _read_i16_le(data: bytes, offset: int) -> int:
    value = data[offset] | (data[offset + 1] << 8)
    if value & 0x8000:
        value -= 0x10000
    return value


def _read_i24_le(data: bytes, offset: int) -> int:
    value = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16)
    if value & 0x800000:
        value -= 0x1000000
    return value


def _parse_frame(frame: bytes) -> str:
    if len(frame) < 6:
        return "帧太短"
    if frame[0] != HEADER or frame[-1] != TAIL:
        return "帧头或帧尾错误"

    length = frame[1]
    expect_len = 1 + 1 + length + 2 + 1
    if len(frame) != expect_len:
        return "帧长度不匹配"

    data = frame[2:2 + length]
    recv_crc = frame[2 + length] | (frame[3 + length] << 8)
    calc_crc = _crc16(bytes([length]) + data)
    if recv_crc != calc_crc:
        return f"CRC错误 calc=0x{calc_crc:04X} recv=0x{recv_crc:04X}"

    if len(data) < 1:
        return "数据区为空"

    func = data[0]
    payload = data[1:]

    if func == FUNC_ENCODER_COUNTS and len(payload) >= 6:
        left = _read_i24_le(payload, 0)
        right = _read_i24_le(payload, 3)
        return f"enc_counts left={left} right={right}"

    if func == FUNC_IMU_BUNDLE and len(payload) >= 18:
        ax = _read_i16_le(payload, 0)
        ay = _read_i16_le(payload, 2)
        az = _read_i16_le(payload, 4)
        gx = _read_i16_le(payload, 6)
        gy = _read_i16_le(payload, 8)
        gz = _read_i16_le(payload, 10)
        roll = _read_i16_le(payload, 12) / 10.0
        pitch = _read_i16_le(payload, 14) / 10.0
        yaw = _read_i16_le(payload, 16) / 10.0
        yaw = yaw % 360.0
        if yaw < 0.0:
            yaw += 360.0
        return f"imu raw=({ax},{ay},{az},{gx},{gy},{gz}) rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})"

    return f"func=0x{func:02X} payload={_hex(payload)}"


class SerialControlApp:
    def __init__(self, root: tk.Tk) -> None:
        self._root = root
        self._root.title("ROS-CAR 串口控制")

        try:
            import serial
            from serial.tools import list_ports
        except Exception:
            messagebox.showerror("缺少依赖", "请先安装 pyserial：pip install pyserial")
            raise

        self._serial_mod = serial
        self._list_ports = list_ports
        self._ser = None
        self._repeat_job = None
        self._reader_stop = threading.Event()
        self._reader_thread = None
        self._rx_buffer = bytearray()
        self._last_counts = None
        self._last_counts_t = None

        self.var_port = tk.StringVar(value="")
        self.var_baud = tk.StringVar(value="115200")
        self.var_mode = tk.StringVar(value="speed")
        self.var_left = tk.StringVar(value="0")
        self.var_right = tk.StringVar(value="0")
        self.var_repeat_ms = tk.StringVar(value="200")

        self._build_ui()
        self._refresh_ports()
        self._update_mode_labels()

    def _build_ui(self) -> None:
        frm = ttk.Frame(self._root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        row = 0
        ttk.Label(frm, text="端口").grid(row=row, column=0, sticky="w")
        self.cb_port = ttk.Combobox(frm, textvariable=self.var_port, width=18, state="readonly")
        self.cb_port.grid(row=row, column=1, sticky="we", padx=(6, 0))
        ttk.Button(frm, text="刷新", command=self._refresh_ports).grid(row=row, column=2, padx=(6, 0))

        row += 1
        ttk.Label(frm, text="波特率").grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(frm, textvariable=self.var_baud, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))
        self.btn_connect = ttk.Button(frm, text="打开串口", command=self._toggle_connect)
        self.btn_connect.grid(row=row, column=2, padx=(6, 0), pady=(6, 0))

        row += 1
        mode_frame = ttk.Frame(frm)
        mode_frame.grid(row=row, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(mode_frame, text="模式").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(mode_frame, text="速度(cm/s)", value="speed", variable=self.var_mode, command=self._update_mode_labels).grid(row=0, column=1, padx=(8, 0))
        ttk.Radiobutton(mode_frame, text="目标计数(counts)", value="counts", variable=self.var_mode, command=self._update_mode_labels).grid(row=0, column=2, padx=(8, 0))

        row += 1
        self.lbl_left = ttk.Label(frm, text="左")
        self.lbl_left.grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(frm, textvariable=self.var_left, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))

        row += 1
        self.lbl_right = ttk.Label(frm, text="右")
        self.lbl_right.grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(frm, textvariable=self.var_right, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))

        row += 1
        ttk.Label(frm, text="循环发送间隔(ms)").grid(row=row, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(frm, textvariable=self.var_repeat_ms, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(10, 0))
        btns = ttk.Frame(frm)
        btns.grid(row=row, column=2, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="发送一次", command=self._send_once).grid(row=0, column=0, padx=(0, 6))
        self.btn_repeat = ttk.Button(btns, text="开始循环", command=self._toggle_repeat)
        self.btn_repeat.grid(row=0, column=1)

        row += 1
        btns2 = ttk.Frame(frm)
        btns2.grid(row=row, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(btns2, text="发送停止", command=self._send_stop).grid(row=0, column=0)

        row += 1
        ttk.Label(frm, text="发送帧(HEX)").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_hex = tk.Text(frm, height=2, width=48)
        self.txt_hex.grid(row=row, column=0, columnspan=3, sticky="we", pady=(6, 0))
        self.txt_hex.configure(state="disabled")

        row += 1
        ttk.Label(frm, text="状态").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_log = tk.Text(frm, height=10, width=48)
        self.txt_log.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(6, 0))

        row += 1
        ttk.Label(frm, text="接收解析").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_rx = tk.Text(frm, height=10, width=48)
        self.txt_rx.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        frm.columnconfigure(1, weight=1)
        frm.rowconfigure(row, weight=1)

    def _log(self, s: str) -> None:
        self.txt_log.insert("end", s + "\n")
        self.txt_log.see("end")

    def _set_hex(self, frame: bytes) -> None:
        self.txt_hex.configure(state="normal")
        self.txt_hex.delete("1.0", "end")
        self.txt_hex.insert("end", _hex(frame))
        self.txt_hex.configure(state="disabled")

    def _log_rx(self, s: str) -> None:
        self.txt_rx.insert("end", s + "\n")
        self.txt_rx.see("end")

    def _refresh_ports(self) -> None:
        ports = [p.device for p in self._list_ports.comports()]
        self.cb_port["values"] = ports
        if ports and (self.var_port.get() not in ports):
            self.var_port.set(ports[0])
        if not ports:
            self.var_port.set("")

    def _toggle_connect(self) -> None:
        if self._ser is not None:
            self._stop_repeat()
            self._stop_reader()
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.btn_connect.configure(text="打开串口")
            self._log("串口已关闭")
            return

        port = self.var_port.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口号")
            return
        try:
            baud = int(self.var_baud.get().strip())
        except Exception:
            messagebox.showwarning("提示", "波特率必须是整数")
            return

        try:
            self._ser = self._serial_mod.Serial(port, baud, timeout=0.05, write_timeout=0.2)
        except Exception as e:
            messagebox.showerror("打开失败", str(e))
            self._ser = None
            return

        self._start_reader()
        self.btn_connect.configure(text="关闭串口")
        self._log(f"串口已打开 {port} @ {baud}")

    def _start_reader(self) -> None:
        self._reader_stop.clear()
        self._rx_buffer.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _stop_reader(self) -> None:
        self._reader_stop.set()
        self._reader_thread = None

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set():
            if self._ser is None:
                return
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                self._root.after(0, lambda: self._log_rx(f"接收失败: {e}"))
                return
            if not chunk:
                continue
            self._rx_buffer.extend(chunk)
            self._consume_rx_buffer()

    def _consume_rx_buffer(self) -> None:
        while True:
            start = self._rx_buffer.find(bytes([HEADER]))
            if start < 0:
                self._rx_buffer.clear()
                return
            if start > 0:
                del self._rx_buffer[:start]

            if len(self._rx_buffer) < 6:
                return

            data_len = self._rx_buffer[1]
            frame_len = 1 + 1 + data_len + 2 + 1
            if len(self._rx_buffer) < frame_len:
                return

            frame = bytes(self._rx_buffer[:frame_len])
            del self._rx_buffer[:frame_len]

            parsed = self._parse_frame_with_speed(frame)
            frame_hex = _hex(frame)
            self._root.after(0, lambda fh=frame_hex, ps=parsed: self._log_rx(f"{fh} | {ps}"))

    def _parse_frame_with_speed(self, frame: bytes) -> str:
        if len(frame) < 6:
            return "帧太短"
        if frame[0] != HEADER or frame[-1] != TAIL:
            return "帧头或帧尾错误"

        length = frame[1]
        expect_len = 1 + 1 + length + 2 + 1
        if len(frame) != expect_len:
            return "帧长度不匹配"

        data = frame[2:2 + length]
        recv_crc = frame[2 + length] | (frame[3 + length] << 8)
        calc_crc = _crc16(bytes([length]) + data)
        if recv_crc != calc_crc:
            return f"CRC错误 calc=0x{calc_crc:04X} recv=0x{recv_crc:04X}"

        if len(data) < 1:
            return "数据区为空"

        func = data[0]
        payload = data[1:]

        if func == FUNC_ENCODER_COUNTS and len(payload) >= 6:
            left = _read_i24_le(payload, 0)
            right = _read_i24_le(payload, 3)

            now = time.monotonic()
            speed_part = ""
            if self._last_counts is not None and self._last_counts_t is not None:
                dt = now - self._last_counts_t
                if dt > 1e-6:
                    dl = left - self._last_counts[0]
                    dr = right - self._last_counts[1]
                    v_l = (dl * WHEEL_CM_PER_COUNT) / dt
                    v_r = (dr * WHEEL_CM_PER_COUNT) / dt
                    speed_part = f" v=({v_l:.2f},{v_r:.2f})cm/s"

            self._last_counts = (left, right)
            self._last_counts_t = now
            return f"enc_counts left={left} right={right}{speed_part}"

        return _parse_frame(frame)

    def _update_mode_labels(self) -> None:
        if self.var_mode.get() == "speed":
            self.lbl_left.configure(text="左(cm/s)")
            self.lbl_right.configure(text="右(cm/s)")
        else:
            self.lbl_left.configure(text="左(counts)")
            self.lbl_right.configure(text="右(counts)")

    def _build_frame_from_ui(self) -> bytes:
        mode = self.var_mode.get()
        if mode == "speed":
            left = float(self.var_left.get().strip())
            right = float(self.var_right.get().strip())
            return build_wheel_speed_frame(left, right)
        left_i = int(float(self.var_left.get().strip()))
        right_i = int(float(self.var_right.get().strip()))
        return build_wheel_target_counts_frame(left_i, right_i)

    def _send_frame(self, frame: bytes) -> None:
        self._set_hex(frame)
        if self._ser is None:
            self._log("未打开串口，只生成帧")
            return
        try:
            self._ser.write(frame)
            self._ser.flush()
            self._log("已发送")
        except Exception as e:
            self._log(f"发送失败: {e}")

    def _send_once(self) -> None:
        try:
            frame = self._build_frame_from_ui()
        except Exception as e:
            messagebox.showerror("参数错误", str(e))
            return
        self._send_frame(frame)

    def _send_stop(self) -> None:
        frame = build_wheel_speed_frame(0.0, 0.0)
        self._send_frame(frame)

    def _repeat_tick(self) -> None:
        self._repeat_job = None
        if self._ser is None:
            self._stop_repeat()
            self._log("串口未打开，已停止循环发送")
            return
        try:
            frame = self._build_frame_from_ui()
            self._send_frame(frame)
        except Exception as e:
            self._log(f"循环发送停止: {e}")
            self._stop_repeat()
            return

        try:
            ms = int(float(self.var_repeat_ms.get().strip()))
        except Exception:
            ms = 200
        ms = max(10, ms)
        self._repeat_job = self._root.after(ms, self._repeat_tick)

    def _stop_repeat(self) -> None:
        if self._repeat_job is not None:
            try:
                self._root.after_cancel(self._repeat_job)
            except Exception:
                pass
            self._repeat_job = None
        self.btn_repeat.configure(text="开始循环")

    def _toggle_repeat(self) -> None:
        if self._repeat_job is not None:
            self._stop_repeat()
            self._log("循环发送已停止")
            return
        self.btn_repeat.configure(text="停止循环")
        self._log("循环发送已开始")
        self._repeat_tick()


def run_gui() -> int:
    root = tk.Tk()
    try:
        SerialControlApp(root)
    except Exception:
        return 2
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(run_gui())