import struct
import sys
import tkinter as tk
from tkinter import messagebox, ttk
import threading
import time
import math
from collections import deque
import bisect

HEADER = 0xAA
TAIL = 0xBB

FUNC_ENCODER_COUNTS = 0x01
FUNC_IMU_BUNDLE = 0x02
FUNC_WHEEL_SPEED = 0x10
FUNC_PID_CONTROL = 0x11

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


def build_pid_control_frame(kp: float, ki: float, kd: float) -> bytes:
    payload = struct.pack("<fff", kp, ki, kd)
    return _build_frame(FUNC_PID_CONTROL, payload)


def _i24_le(v: int) -> bytes:
    v = max(-8388608, min(8388607, int(v)))
    if v < 0:
        v = (1 << 24) + v
    return bytes([(v >> 0) & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])




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
        speed_part = ""
        if len(payload) >= 10:
            left_speed = _read_i16_le(payload, 6) / 100.0
            right_speed = _read_i16_le(payload, 8) / 100.0
            speed_part = f" v=({left_speed:.2f},{right_speed:.2f})cm/s"
        pwm_part = ""
        if len(payload) >= 14:
            left_pwm = _read_i16_le(payload, 10)
            right_pwm = _read_i16_le(payload, 12)
            pwm_part = f" pwm=({left_pwm},{right_pwm})"
        return f"enc_counts left={left} right={right}{speed_part}{pwm_part}"

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

        self._plot_frame = None
        self._plot_visible = False
        self._plot_controls = None
        self._plot_left = None
        self._plot_canvas_host = None
        self._plot_right = None
        self._plot_canvas = None
        self._plot_fig = None
        self._plot_ax = None
        self._plot_line_target_l = None
        self._plot_line_target_r = None
        self._plot_line_actual_l = None
        self._plot_line_actual_r = None
        self._plot_line_pwm_l = None
        self._plot_line_pwm_r = None
        self._plot_lines = {}
        self._plot_line_colors = {}
        self._plot_fade_jobs = {}
        self._plot_job = None
        self._mpl_cid_motion = None
        self._mpl_cid_leave = None
        self._plot_tip = None
        self._plot_tip_fade_job = None
        self._plot_t0 = time.monotonic()
        self._plot_target_t = deque(maxlen=3000)
        self._plot_target_l = deque(maxlen=3000)
        self._plot_target_r = deque(maxlen=3000)
        self._plot_actual_t = deque(maxlen=3000)
        self._plot_actual_l = deque(maxlen=3000)
        self._plot_actual_r = deque(maxlen=3000)
        self._plot_pwm_t = deque(maxlen=3000)
        self._plot_pwm_l = deque(maxlen=3000)
        self._plot_pwm_r = deque(maxlen=3000)
        self._plot_var_target_l = tk.BooleanVar(value=True)
        self._plot_var_target_r = tk.BooleanVar(value=True)
        self._plot_var_actual_l = tk.BooleanVar(value=True)
        self._plot_var_actual_r = tk.BooleanVar(value=True)
        self._plot_var_pwm_l = tk.BooleanVar(value=True)
        self._plot_var_pwm_r = tk.BooleanVar(value=True)
        self._plot_chk_target_l = None
        self._plot_chk_target_r = None
        self._plot_chk_actual_l = None
        self._plot_chk_actual_r = None
        self._plot_chk_pwm_l = None
        self._plot_chk_pwm_r = None
        self._plot_lbl_t = None
        self._plot_lbl_target_l = None
        self._plot_lbl_target_r = None
        self._plot_lbl_actual_l = None
        self._plot_lbl_actual_r = None
        self._plot_lbl_pwm_l = None
        self._plot_lbl_pwm_r = None

        self.var_port = tk.StringVar(value="")
        self.var_baud = tk.StringVar(value="115200")
        self.var_mode = tk.StringVar(value="speed")
        self.var_left = tk.StringVar(value="0")
        self.var_right = tk.StringVar(value="0")
        self.var_repeat_ms = tk.StringVar(value="200")
        self.var_pid_kp = tk.StringVar(value="1.2")
        self.var_pid_ki = tk.StringVar(value="0.25")
        self.var_pid_kd = tk.StringVar(value="0.0")

        self._build_ui()
        self._refresh_ports()
        self._update_mode_labels()

    def _build_ui(self) -> None:
        self._root.columnconfigure(0, weight=1)
        self._root.rowconfigure(0, weight=1)

        frm = ttk.Frame(self._root, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        frm.columnconfigure(0, weight=3)
        frm.columnconfigure(1, weight=2)
        frm.rowconfigure(0, weight=1)

        left = ttk.Frame(frm)
        left.grid(row=0, column=0, sticky="nsew")
        right = ttk.Frame(frm)
        right.grid(row=0, column=1, sticky="nsew", padx=(10, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        row = 0
        ttk.Label(left, text="端口").grid(row=row, column=0, sticky="w")
        self.cb_port = ttk.Combobox(left, textvariable=self.var_port, width=18, state="readonly")
        self.cb_port.grid(row=row, column=1, sticky="we", padx=(6, 0))
        ttk.Button(left, text="刷新", command=self._refresh_ports).grid(row=row, column=2, padx=(6, 0))

        row += 1
        ttk.Label(left, text="波特率").grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(left, textvariable=self.var_baud, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))
        self.btn_connect = ttk.Button(left, text="打开串口", command=self._toggle_connect)
        self.btn_connect.grid(row=row, column=2, padx=(6, 0), pady=(6, 0))

        row += 1
        mode_frame = ttk.Frame(left)
        mode_frame.grid(row=row, column=0, columnspan=3, sticky="w", pady=(10, 0))
        ttk.Label(mode_frame, text="模式").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(mode_frame, text="速度(cm/s)", value="speed", variable=self.var_mode, command=self._update_mode_labels).grid(row=0, column=1, padx=(8, 0))
        self.var_mode.set("speed")

        row += 1
        self.lbl_left = ttk.Label(left, text="左")
        self.lbl_left.grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(left, textvariable=self.var_left, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))

        row += 1
        self.lbl_right = ttk.Label(left, text="右")
        self.lbl_right.grid(row=row, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(left, textvariable=self.var_right, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(6, 0))

        row += 1
        ttk.Label(left, text="循环发送间隔(ms)").grid(row=row, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(left, textvariable=self.var_repeat_ms, width=18).grid(row=row, column=1, sticky="we", padx=(6, 0), pady=(10, 0))
        btns = ttk.Frame(left)
        btns.grid(row=row, column=2, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="发送一次", command=self._send_once).grid(row=0, column=0, padx=(0, 6))
        self.btn_repeat = ttk.Button(btns, text="开始循环", command=self._toggle_repeat)
        self.btn_repeat.grid(row=0, column=1)

        row += 1
        btns2 = ttk.Frame(left)
        btns2.grid(row=row, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(btns2, text="发送停止", command=self._send_stop).grid(row=0, column=0)

        row += 1
        pid_frame = ttk.LabelFrame(left, text="PID_control")
        pid_frame.grid(row=row, column=0, columnspan=3, sticky="we", pady=(10, 0))
        pid_frame.columnconfigure(1, weight=1)
        pid_frame.columnconfigure(3, weight=1)
        pid_frame.columnconfigure(5, weight=1)
        ttk.Label(pid_frame, text="Kp").grid(row=0, column=0, sticky="w", padx=(6, 0), pady=6)
        ttk.Entry(pid_frame, textvariable=self.var_pid_kp, width=8).grid(row=0, column=1, sticky="we", padx=(4, 8), pady=6)
        ttk.Label(pid_frame, text="Ki").grid(row=0, column=2, sticky="w", pady=6)
        ttk.Entry(pid_frame, textvariable=self.var_pid_ki, width=8).grid(row=0, column=3, sticky="we", padx=(4, 8), pady=6)
        ttk.Label(pid_frame, text="Kd").grid(row=0, column=4, sticky="w", pady=6)
        ttk.Entry(pid_frame, textvariable=self.var_pid_kd, width=8).grid(row=0, column=5, sticky="we", padx=(4, 8), pady=6)
        ttk.Button(pid_frame, text="发送PID", command=self._send_pid_control).grid(row=0, column=6, padx=(0, 6), pady=6)

        row += 1
        ttk.Label(left, text="发送帧(HEX)").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_hex = tk.Text(left, height=2, width=48)
        self.txt_hex.grid(row=row, column=0, columnspan=3, sticky="we", pady=(6, 0))
        self.txt_hex.configure(state="disabled")

        row += 1
        ttk.Label(left, text="状态").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_log = tk.Text(left, height=10, width=48)
        self.txt_log.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        log_row = row

        row += 1
        ttk.Label(left, text="接收解析").grid(row=row, column=0, sticky="w", pady=(10, 0))
        self.txt_rx = tk.Text(left, height=10, width=48)
        self.txt_rx.grid(row=row, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        rx_row = row
        left.columnconfigure(1, weight=1)
        left.rowconfigure(log_row, weight=1)
        left.rowconfigure(rx_row, weight=1)

        self._plot_frame = ttk.Frame(right)
        self._plot_frame.grid(row=0, column=0, sticky="nsew")
        self._plot_frame.grid_remove()

        body = ttk.Frame(self._plot_frame)
        body.pack(fill="both", expand=True)

        self._plot_left = ttk.Frame(body)
        self._plot_left.pack(side="left", fill="both", expand=True)
        self._plot_right = ttk.Frame(body, width=180)
        self._plot_right.pack(side="right", fill="y", expand=False, padx=(10, 0))

        self._plot_controls = ttk.Frame(self._plot_left)
        self._plot_controls.pack(fill="x", expand=False)
        self._plot_canvas_host = ttk.Frame(self._plot_left)
        self._plot_canvas_host.pack(fill="both", expand=True)

        self._build_plot_value_panel()

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
            if len(payload) >= 10:
                v_l = _read_i16_le(payload, 6) / 100.0
                v_r = _read_i16_le(payload, 8) / 100.0
                speed_part = f" v=({v_l:.2f},{v_r:.2f})cm/s"
                self._record_actual_speed(v_l, v_r, now)
            elif self._last_counts is not None and self._last_counts_t is not None:
                dt = now - self._last_counts_t
                if dt > 1e-6:
                    dl = left - self._last_counts[0]
                    dr = right - self._last_counts[1]
                    v_l = (dl * WHEEL_CM_PER_COUNT) / dt
                    v_r = (dr * WHEEL_CM_PER_COUNT) / dt
                    speed_part = f" v=({v_l:.2f},{v_r:.2f})cm/s"
                    self._record_actual_speed(v_l, v_r, now)

            self._last_counts = (left, right)
            self._last_counts_t = now
            pwm_part = ""
            if len(payload) >= 14:
                pwm_l = _read_i16_le(payload, 10)
                pwm_r = _read_i16_le(payload, 12)
                pwm_part = f" pwm=({pwm_l},{pwm_r})"
                self._record_pwm(pwm_l, pwm_r, now)
            return f"enc_counts left={left} right={right}{speed_part}{pwm_part}"

        return _parse_frame(frame)

    def _update_mode_labels(self) -> None:
        self.lbl_left.configure(text="左(cm/s)")
        self.lbl_right.configure(text="右(cm/s)")

    def _build_frame_from_ui(self) -> bytes:
        left = float(self.var_left.get().strip())
        right = float(self.var_right.get().strip())
        return build_wheel_speed_frame(left, right)

    def _record_target_speed(self, left_cm_s: float, right_cm_s: float) -> None:
        t = time.monotonic() - self._plot_t0
        self._plot_target_t.append(t)
        self._plot_target_l.append(left_cm_s)
        self._plot_target_r.append(right_cm_s)

    def _record_actual_speed(self, left_cm_s: float, right_cm_s: float, now: float | None = None) -> None:
        if now is None:
            now = time.monotonic()
        t = now - self._plot_t0
        self._plot_actual_t.append(t)
        self._plot_actual_l.append(left_cm_s)
        self._plot_actual_r.append(right_cm_s)

    def _record_pwm(self, left_pwm: int, right_pwm: int, now: float | None = None) -> None:
        if now is None:
            now = time.monotonic()
        t = now - self._plot_t0
        self._plot_pwm_t.append(t)
        self._plot_pwm_l.append(left_pwm)
        self._plot_pwm_r.append(right_pwm)

    def _show_plot(self) -> None:
        if self._plot_frame is None:
            return
        if not self._ensure_plot():
            return
        if not self._plot_visible:
            self._plot_frame.grid()
            self._plot_visible = True
        if self._plot_job is None:
            self._plot_tick()

    def _hide_plot(self) -> None:
        if self._plot_job is not None:
            try:
                self._root.after_cancel(self._plot_job)
            except Exception:
                pass
            self._plot_job = None
        for _, job in list(self._plot_fade_jobs.items()):
            try:
                self._root.after_cancel(job)
            except Exception:
                pass
        self._plot_fade_jobs.clear()
        if self._plot_tip_fade_job is not None:
            try:
                self._root.after_cancel(self._plot_tip_fade_job)
            except Exception:
                pass
            self._plot_tip_fade_job = None
        if self._plot_tip is not None:
            self._plot_tip.set_visible(False)
            self._plot_tip.set_alpha(0.0)
        if self._plot_frame is not None:
            self._plot_frame.grid_remove()
        self._plot_visible = False

    def _ensure_plot(self) -> bool:
        if self._plot_frame is None or self._plot_canvas_host is None:
            return False
        if self._plot_canvas is not None:
            return True
        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except Exception:
            messagebox.showerror("缺少依赖", "请先安装 matplotlib：pip install matplotlib")
            return False

        fig = Figure(figsize=(7.0, 4.0), dpi=100)
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlabel("t(s)")
        ax.set_ylabel("cm/s")
        ax.grid(True)

        (line_tl,) = ax.plot([], [], linestyle="--", linewidth=1.5, label="target L")
        (line_tr,) = ax.plot([], [], linestyle="--", linewidth=1.5, label="target R")
        (line_al,) = ax.plot([], [], linewidth=2.0, label="actual L")
        (line_ar,) = ax.plot([], [], linewidth=2.0, label="actual R")
        (line_pl,) = ax.plot([], [], linestyle=":", linewidth=1.8, label="pwm L")
        (line_pr,) = ax.plot([], [], linestyle=":", linewidth=1.8, label="pwm R")

        canvas = FigureCanvasTkAgg(fig, master=self._plot_canvas_host)
        canvas.get_tk_widget().pack(fill="both", expand=True)

        tip = ax.annotate(
            "",
            xy=(0, 0),
            xytext=(12, 12),
            textcoords="offset points",
            bbox=dict(boxstyle="round", fc="white", ec="#666666", alpha=0.9),
        )
        tip.set_visible(False)
        tip.set_alpha(0.0)

        self._plot_fig = fig
        self._plot_ax = ax
        self._plot_canvas = canvas
        self._plot_line_target_l = line_tl
        self._plot_line_target_r = line_tr
        self._plot_line_actual_l = line_al
        self._plot_line_actual_r = line_ar
        self._plot_line_pwm_l = line_pl
        self._plot_line_pwm_r = line_pr
        self._plot_lines = {
            "target_l": line_tl,
            "target_r": line_tr,
            "actual_l": line_al,
            "actual_r": line_ar,
            "pwm_l": line_pl,
            "pwm_r": line_pr,
        }
        self._plot_line_colors = {k: v.get_color() for k, v in self._plot_lines.items()}
        self._plot_tip = tip

        self._build_plot_controls()
        self._apply_plot_visibility(smooth=False)
        self._bind_plot_events()
        return True

    def _bind_plot_events(self) -> None:
        if self._plot_canvas is None:
            return
        mpl_canvas = self._plot_canvas
        if self._mpl_cid_motion is None:
            self._mpl_cid_motion = mpl_canvas.mpl_connect("motion_notify_event", self._on_plot_motion)
        if self._mpl_cid_leave is None:
            self._mpl_cid_leave = mpl_canvas.mpl_connect("axes_leave_event", self._on_plot_leave)

    def _build_plot_value_panel(self) -> None:
        if self._plot_right is None:
            return
        for child in self._plot_right.winfo_children():
            child.destroy()

        ttk.Label(self._plot_right, text="数值").pack(anchor="w")
        self._plot_lbl_t = ttk.Label(self._plot_right, text="t: -")
        self._plot_lbl_t.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_target_l = ttk.Label(self._plot_right, text="target L: -")
        self._plot_lbl_target_l.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_target_r = ttk.Label(self._plot_right, text="target R: -")
        self._plot_lbl_target_r.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_actual_l = ttk.Label(self._plot_right, text="actual L: -")
        self._plot_lbl_actual_l.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_actual_r = ttk.Label(self._plot_right, text="actual R: -")
        self._plot_lbl_actual_r.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_pwm_l = ttk.Label(self._plot_right, text="pwm L: -")
        self._plot_lbl_pwm_l.pack(anchor="w", pady=(6, 0))
        self._plot_lbl_pwm_r = ttk.Label(self._plot_right, text="pwm R: -")
        self._plot_lbl_pwm_r.pack(anchor="w", pady=(6, 0))

    def _nearest_value(self, t_list: list[float], y_list: list[float], t: float) -> float | None:
        if not t_list:
            return None
        i = bisect.bisect_left(t_list, t)
        if i <= 0:
            return y_list[0] if y_list else None
        if i >= len(t_list):
            return y_list[-1] if y_list else None
        if abs(t_list[i] - t) < abs(t_list[i - 1] - t):
            return y_list[i] if i < len(y_list) else None
        return y_list[i - 1] if (i - 1) < len(y_list) else None

    def _on_plot_motion(self, event) -> None:
        if not self._plot_visible:
            return
        if self._plot_ax is None or self._plot_canvas is None:
            return
        if event.inaxes != self._plot_ax or event.xdata is None:
            self._plot_hide_tip()
            return

        t = float(event.xdata)
        tl_t = list(self._plot_target_t)
        tl_y = list(self._plot_target_l)
        tr_y = list(self._plot_target_r)
        al_t = list(self._plot_actual_t)
        al_y = list(self._plot_actual_l)
        ar_y = list(self._plot_actual_r)
        pwm_t = list(self._plot_pwm_t)
        pwm_l_y = list(self._plot_pwm_l)
        pwm_r_y = list(self._plot_pwm_r)

        v_tl = self._nearest_value(tl_t, tl_y, t) if self._plot_var_target_l.get() else None
        v_tr = self._nearest_value(tl_t, tr_y, t) if self._plot_var_target_r.get() else None
        v_al = self._nearest_value(al_t, al_y, t) if self._plot_var_actual_l.get() else None
        v_ar = self._nearest_value(al_t, ar_y, t) if self._plot_var_actual_r.get() else None
        v_pl = self._nearest_value(pwm_t, pwm_l_y, t) if self._plot_var_pwm_l.get() else None
        v_pr = self._nearest_value(pwm_t, pwm_r_y, t) if self._plot_var_pwm_r.get() else None

        if self._plot_lbl_t is not None:
            self._plot_lbl_t.configure(text=f"t: {t:.2f} s")
        if self._plot_lbl_target_l is not None:
            self._plot_lbl_target_l.configure(text=("target L: -" if v_tl is None else f"target L: {v_tl:.2f} cm/s"))
        if self._plot_lbl_target_r is not None:
            self._plot_lbl_target_r.configure(text=("target R: -" if v_tr is None else f"target R: {v_tr:.2f} cm/s"))
        if self._plot_lbl_actual_l is not None:
            self._plot_lbl_actual_l.configure(text=("actual L: -" if v_al is None else f"actual L: {v_al:.2f} cm/s"))
        if self._plot_lbl_actual_r is not None:
            self._plot_lbl_actual_r.configure(text=("actual R: -" if v_ar is None else f"actual R: {v_ar:.2f} cm/s"))
        if self._plot_lbl_pwm_l is not None:
            self._plot_lbl_pwm_l.configure(text=("pwm L: -" if v_pl is None else f"pwm L: {v_pl:.0f}"))
        if self._plot_lbl_pwm_r is not None:
            self._plot_lbl_pwm_r.configure(text=("pwm R: -" if v_pr is None else f"pwm R: {v_pr:.0f}"))

        tip_lines = [f"t={t:.2f}s"]
        if v_tl is not None:
            tip_lines.append(f"target L {v_tl:.2f}")
        if v_tr is not None:
            tip_lines.append(f"target R {v_tr:.2f}")
        if v_al is not None:
            tip_lines.append(f"actual L {v_al:.2f}")
        if v_ar is not None:
            tip_lines.append(f"actual R {v_ar:.2f}")
        if v_pl is not None:
            tip_lines.append(f"pwm L {v_pl:.0f}")
        if v_pr is not None:
            tip_lines.append(f"pwm R {v_pr:.0f}")

        self._plot_show_tip("\n".join(tip_lines), event)

    def _on_plot_leave(self, _event) -> None:
        self._plot_hide_tip()

    def _plot_show_tip(self, text: str, event) -> None:
        if self._plot_tip is None or self._plot_ax is None:
            return
        self._plot_tip.set_text(text)
        self._plot_tip.xy = (event.xdata, event.ydata if event.ydata is not None else 0.0)

        x_pix = float(event.x)
        y_pix = float(event.y)
        w = float(self._plot_canvas.get_tk_widget().winfo_width()) if self._plot_canvas is not None else 1.0
        h = float(self._plot_canvas.get_tk_widget().winfo_height()) if self._plot_canvas is not None else 1.0
        dx = 12
        dy = 12
        if x_pix > w * 0.75:
            dx = -120
        if y_pix > h * 0.75:
            dy = -60
        self._plot_tip.set_position((dx, dy))

        self._plot_tip.set_visible(True)
        self._plot_tip_fade(to_alpha=1.0)

    def _plot_hide_tip(self) -> None:
        if self._plot_tip is None:
            return
        self._plot_tip_fade(to_alpha=0.0, hide_after=True)

    def _plot_tip_fade(self, to_alpha: float, hide_after: bool = False) -> None:
        if self._plot_canvas is None or self._plot_tip is None:
            return
        if self._plot_tip_fade_job is not None:
            try:
                self._root.after_cancel(self._plot_tip_fade_job)
            except Exception:
                pass
            self._plot_tip_fade_job = None

        steps = 10
        step_ms = 30
        start_alpha = float(self._plot_tip.get_alpha() if self._plot_tip.get_alpha() is not None else 0.0)

        def tick(i: int) -> None:
            t = i / float(steps)
            alpha = (start_alpha * (1.0 - t)) + (to_alpha * t)
            self._plot_tip.set_alpha(alpha)
            if self._plot_canvas is not None:
                self._plot_canvas.draw_idle()
            if i >= steps:
                self._plot_tip_fade_job = None
                if hide_after and to_alpha <= 0.0:
                    self._plot_tip.set_visible(False)
                    self._plot_tip.set_alpha(0.0)
                    if self._plot_canvas is not None:
                        self._plot_canvas.draw_idle()
                return
            self._plot_tip_fade_job = self._root.after(step_ms, lambda: tick(i + 1))

        tick(0)

    def _build_plot_controls(self) -> None:
        if self._plot_controls is None:
            return
        for child in self._plot_controls.winfo_children():
            child.destroy()

        tk.Label(self._plot_controls, text="图例").pack(side="left")
        tk.Button(self._plot_controls, text="全选", command=lambda: self._plot_set_all(True)).pack(side="right")
        tk.Button(self._plot_controls, text="全不选", command=lambda: self._plot_set_all(False)).pack(side="right", padx=(6, 0))

        self._plot_chk_target_l = tk.Checkbutton(
            self._plot_controls,
            text="target L",
            variable=self._plot_var_target_l,
            command=self._on_plot_toggle,
        )
        self._plot_chk_target_r = tk.Checkbutton(
            self._plot_controls,
            text="target R",
            variable=self._plot_var_target_r,
            command=self._on_plot_toggle,
        )
        self._plot_chk_actual_l = tk.Checkbutton(
            self._plot_controls,
            text="actual L",
            variable=self._plot_var_actual_l,
            command=self._on_plot_toggle,
        )
        self._plot_chk_actual_r = tk.Checkbutton(
            self._plot_controls,
            text="actual R",
            variable=self._plot_var_actual_r,
            command=self._on_plot_toggle,
        )
        self._plot_chk_pwm_l = tk.Checkbutton(
            self._plot_controls,
            text="pwm L",
            variable=self._plot_var_pwm_l,
            command=self._on_plot_toggle,
        )
        self._plot_chk_pwm_r = tk.Checkbutton(
            self._plot_controls,
            text="pwm R",
            variable=self._plot_var_pwm_r,
            command=self._on_plot_toggle,
        )

        self._plot_chk_target_l.pack(side="left", padx=(8, 0))
        self._plot_chk_target_r.pack(side="left", padx=(8, 0))
        self._plot_chk_actual_l.pack(side="left", padx=(8, 0))
        self._plot_chk_actual_r.pack(side="left", padx=(8, 0))
        self._plot_chk_pwm_l.pack(side="left", padx=(8, 0))
        self._plot_chk_pwm_r.pack(side="left", padx=(8, 0))
        self._update_plot_control_colors()

    def _plot_set_all(self, visible: bool) -> None:
        self._plot_var_target_l.set(visible)
        self._plot_var_target_r.set(visible)
        self._plot_var_actual_l.set(visible)
        self._plot_var_actual_r.set(visible)
        self._plot_var_pwm_l.set(visible)
        self._plot_var_pwm_r.set(visible)
        self._apply_plot_visibility(smooth=True)

    def _on_plot_toggle(self) -> None:
        self._apply_plot_visibility(smooth=True)

    def _apply_plot_visibility(self, smooth: bool) -> None:
        desired = {
            "target_l": bool(self._plot_var_target_l.get()),
            "target_r": bool(self._plot_var_target_r.get()),
            "actual_l": bool(self._plot_var_actual_l.get()),
            "actual_r": bool(self._plot_var_actual_r.get()),
            "pwm_l": bool(self._plot_var_pwm_l.get()),
            "pwm_r": bool(self._plot_var_pwm_r.get()),
        }

        for key, want_visible in desired.items():
            line = self._plot_lines.get(key)
            if line is None:
                continue
            if smooth:
                self._fade_line(key, want_visible)
            else:
                line.set_alpha(1.0)
                line.set_visible(want_visible)

        self._update_plot_control_colors()
        if self._plot_canvas is not None:
            self._plot_canvas.draw_idle()

    def _fade_line(self, key: str, want_visible: bool) -> None:
        if self._plot_canvas is None:
            return
        line = self._plot_lines.get(key)
        if line is None:
            return
        was_visible = bool(line.get_visible())

        prev_job = self._plot_fade_jobs.get(key)
        if prev_job is not None:
            try:
                self._root.after_cancel(prev_job)
            except Exception:
                pass
            self._plot_fade_jobs.pop(key, None)

        steps = 8
        step_ms = 20
        start_alpha = float(line.get_alpha() if line.get_alpha() is not None else 1.0)
        if want_visible:
            line.set_visible(True)
            end_alpha = 1.0
            if not was_visible:
                start_alpha = 0.0
        else:
            end_alpha = 0.0

        def tick(i: int) -> None:
            t = i / float(steps)
            alpha = (start_alpha * (1.0 - t)) + (end_alpha * t)
            line.set_alpha(alpha)
            if self._plot_canvas is not None:
                self._plot_canvas.draw_idle()
            if i >= steps:
                if not want_visible:
                    line.set_visible(False)
                    line.set_alpha(1.0)
                    if self._plot_canvas is not None:
                        self._plot_canvas.draw_idle()
                self._plot_fade_jobs.pop(key, None)
                self._update_plot_control_colors()
                return
            self._plot_fade_jobs[key] = self._root.after(step_ms, lambda: tick(i + 1))

        tick(0)

    def _update_plot_control_colors(self) -> None:
        mapping = [
            ("target_l", self._plot_chk_target_l),
            ("target_r", self._plot_chk_target_r),
            ("actual_l", self._plot_chk_actual_l),
            ("actual_r", self._plot_chk_actual_r),
            ("pwm_l", self._plot_chk_pwm_l),
            ("pwm_r", self._plot_chk_pwm_r),
        ]
        for key, chk in mapping:
            if chk is None:
                continue
            line = self._plot_lines.get(key)
            if line is None or not line.get_visible():
                chk.configure(fg="gray")
            else:
                chk.configure(fg=self._plot_line_colors.get(key, "black"))

    def _plot_tick(self) -> None:
        self._plot_job = None
        if not self._plot_visible or self._plot_canvas is None:
            return

        tl_t = list(self._plot_target_t)
        tl_y = list(self._plot_target_l)
        tr_y = list(self._plot_target_r)
        al_t = list(self._plot_actual_t)
        al_y = list(self._plot_actual_l)
        ar_y = list(self._plot_actual_r)
        pwm_t = list(self._plot_pwm_t)
        pwm_l_y = list(self._plot_pwm_l)
        pwm_r_y = list(self._plot_pwm_r)

        if self._plot_line_target_l is not None:
            self._plot_line_target_l.set_data(tl_t, tl_y)
        if self._plot_line_target_r is not None:
            self._plot_line_target_r.set_data(tl_t, tr_y)
        if self._plot_line_actual_l is not None:
            self._plot_line_actual_l.set_data(al_t, al_y)
        if self._plot_line_actual_r is not None:
            self._plot_line_actual_r.set_data(al_t, ar_y)
        if self._plot_line_pwm_l is not None:
            self._plot_line_pwm_l.set_data(pwm_t, pwm_l_y)
        if self._plot_line_pwm_r is not None:
            self._plot_line_pwm_r.set_data(pwm_t, pwm_r_y)

        ax = self._plot_ax
        if ax is not None:
            all_t = []
            all_y = []
            if self._plot_line_target_l is not None and self._plot_line_target_l.get_visible():
                all_t += tl_t
                all_y += tl_y
            if self._plot_line_target_r is not None and self._plot_line_target_r.get_visible():
                all_t += tl_t
                all_y += tr_y
            if self._plot_line_actual_l is not None and self._plot_line_actual_l.get_visible():
                all_t += al_t
                all_y += al_y
            if self._plot_line_actual_r is not None and self._plot_line_actual_r.get_visible():
                all_t += al_t
                all_y += ar_y
            if self._plot_line_pwm_l is not None and self._plot_line_pwm_l.get_visible():
                all_t += pwm_t
                all_y += pwm_l_y
            if self._plot_line_pwm_r is not None and self._plot_line_pwm_r.get_visible():
                all_t += pwm_t
                all_y += pwm_r_y
            if all_t:
                t_min = max(0.0, max(all_t) - 20.0)
                t_max = max(all_t)
                ax.set_xlim(t_min, t_max if t_max > t_min + 1e-6 else t_min + 1.0)
            if all_y:
                y_min = min(all_y)
                y_max = max(all_y)
                pad = max(1.0, 0.05 * (y_max - y_min))
                ax.set_ylim(y_min - pad, y_max + pad)

        self._plot_canvas.draw_idle()
        self._plot_job = self._root.after(100, self._plot_tick)

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
            left = float(self.var_left.get().strip())
            right = float(self.var_right.get().strip())
            self._record_target_speed(left, right)
            frame = self._build_frame_from_ui()
        except Exception as e:
            messagebox.showerror("参数错误", str(e))
            return
        self._send_frame(frame)

    def _send_stop(self) -> None:
        self._record_target_speed(0.0, 0.0)
        frame = build_wheel_speed_frame(0.0, 0.0)
        self._send_frame(frame)

    def _send_pid_control(self) -> None:
        try:
            kp = float(self.var_pid_kp.get().strip())
            ki = float(self.var_pid_ki.get().strip())
            kd = float(self.var_pid_kd.get().strip())
            frame = build_pid_control_frame(kp, ki, kd)
        except Exception as e:
            messagebox.showerror("PID参数错误", str(e))
            return
        self._send_frame(frame)
        self._log(f"PID_control Kp={kp:.4g} Ki={ki:.4g} Kd={kd:.4g}")

    def _repeat_tick(self) -> None:
        self._repeat_job = None
        if self._ser is None:
            self._stop_repeat()
            self._log("串口未打开，已停止循环发送")
            return
        try:
            left = float(self.var_left.get().strip())
            right = float(self.var_right.get().strip())
            self._record_target_speed(left, right)
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
        self._hide_plot()

    def _toggle_repeat(self) -> None:
        if self._repeat_job is not None:
            self._stop_repeat()
            self._log("循环发送已停止")
            return
        self.btn_repeat.configure(text="停止循环")
        self._log("循环发送已开始")
        self._show_plot()
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
