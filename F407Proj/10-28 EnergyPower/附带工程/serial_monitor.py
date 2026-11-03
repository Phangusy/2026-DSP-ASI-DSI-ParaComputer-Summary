# -*- coding: utf-8 -*-
"""
==============================================================================
 serial_monitor.py —— STM32 功率计上位机（串口实时波形 · 分图版）
==============================================================================
 配套工程：D:\\F407Projects\\10-28 EnergyPower
 板子每约 10.7 ms 发一行 CSV，格式固定 7 列：

     N,E,Pwin,Pexp,Pall,RMSwin,RMSexp

   N       累计采样点数（除以采样率就是开机秒数）
   E       累积能量 Σx²（归一化，单调增长）
   Pwin    1024 点滑动窗口平均功率（归一化）
   Pexp    指数平均功率（归一化）
   Pall    总平均功率（归一化）
   RMSwin  滑动窗口 RMS（归一化 0~1）
   RMSexp  指数平均 RMS（归一化 0~1）

 这一版把五条曲线拆成五个独立的图，各自独立 y 轴、各自显示当前读数，
 双击任意一个图可以弹出该曲线的放大窗口。
 Pwin/RMSwin 用同一种蓝色、Pexp/RMSexp 用同一种橙色 —— 因为它们是
 同一个估计器的两种表示（一个是功率、一个是它开根号）。

 归一化怎么变回实际电压：
    输入 Vrms = RMSwin × 1.65 V        （1.65 V 是 ADC 的直流中心）
    PA4 DAC 电压 = RMSwin × 3.3 V      （正好是输入 Vrms 的两倍）
  所以示波器上读到的 PA4 电压 ÷ 2 = 这个程序里"输入 Vrms"那一格。

 用法：
   python serial_monitor.py

 依赖：pyserial、numpy、matplotlib（tkinter 是自带的）

 接线（两种都行，PC 上都是一个 COM 口）：
   A. 板载 ST-LINK 虚拟串口：一根 USB 线插板子的 USB 口
   B. 外接 CH340：F407 PA2(TX)→CH340 RXD，PA3(RX)→CH340 TXD，GND↔GND
==============================================================================
"""

import time
from collections import deque

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import tkinter.font as tkfont

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None

# 中文字体（Windows 上微软雅黑一定有）
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


# ---------------------------------------------------------------- 参数配置 ----
BAUD_CHOICES = ["115200", "230400", "460800", "921600", "57600", "38400"]
DEFAULT_BAUD = "115200"

DEFAULT_FS = 48000.0      # 采样率，单位 Hz（跟 Sample.h 的 SAMPLE_FS_HZ 一致）
DEFAULT_VMID = 1.65       # ADC 直流中心，单位 V（对应码值 2048）
DEFAULT_VDDA = 3.3        # DAC 满量程，单位 V

HISTORY_ROWS = 1500       # 保留最近多少行用于画图（约 16 秒 @94 行/秒）
PLOT_INTERVAL_MS = 200    # 重绘周期，别太快，否则界面卡
POLL_INTERVAL_MS = 40     # 读串口的周期

# 串口读取的硬性上限：这几个数字是防止"一点连接就卡死"的关键。
# 直接在 Tk 主线程里 while 循环 + readline 抽干缓冲的话，一旦串口上没有换行符
# （波特率不匹配时收到的是 0xE0/0x1C 这类字节，一个 \n 都没有），readline 会一直
# 等不到换行，主线程被占死，整个窗口失去响应、事件队列全部堵住。
# 现在改成"一次最多搬多少、最多处理多少行"，剩下的留到下一 tick。
MAX_BYTES_PER_POLL = 8192   # 单次最多从串口搬这么多字节
MAX_LINES_PER_POLL = 200    # 单次最多处理这么多行（多余的留到下一 tick）
MAX_LINE_LEN = 512          # 单行上限，超长当垃圾丢掉
NO_NEWLINE_LIMIT = 8192     # 累计这么多字节还没见到换行 → 判定参数不对，直接报错
RX_SILENCE_SEC = 3.0        # 连接后这么久一个字节都没收到 → 提示接线

# CSV 里 7 列的名字，顺序固定
COLUMNS = ["N", "E", "Pwin", "Pexp", "Pall", "RMSwin", "RMSexp"]

# 五个独立图表的元信息。顺序 = 从上到下的显示顺序。
#   color  曲线颜色（同一估计器的功率与 RMS 用同色，便于对应）
#   title  标题，顺手把换算关系写在标题里
#   ylabel 纵轴标签
#   fmt    右上角当前读数的格式
#   unit   当前读数的单位后缀
CHART_ORDER = ["Pwin", "Pexp", "Pall", "RMSwin", "RMSexp"]

CHART_META = {
    "Pwin": dict(color="#1a73e8", fmt="%.6f", unit="",
                 title="Pwin · 滑动窗口平均功率　（窗口 1024 点 = 21.33 ms @48 kHz）",
                 ylabel="归一化功率 x_rms²"),
    "Pexp": dict(color="#e8710a", fmt="%.6f", unit="",
                 title="Pexp · 指数平均功率　（α = 0.99，τ ≈ 2.07 ms）",
                 ylabel="归一化功率 x_rms²"),
    "Pall": dict(color="#188038", fmt="%.6f", unit="",
                 title="Pall · 总平均功率　（E / N，从开机至今，几乎不响应变化）",
                 ylabel="归一化功率 x_rms²"),
    "RMSwin": dict(color="#1a73e8", fmt="%.4f", unit="",
                   title="RMSwin · 滑动窗口 RMS　（× 1.65 V = 输入 Vrms，× 3.3 V = PA4 电压）",
                   ylabel="归一化 RMS (0~1)"),
    "RMSexp": dict(color="#e8710a", fmt="%.4f", unit="",
                   title="RMSexp · 指数平均 RMS　（× 1.65 V = 输入 Vrms，纹波比上面那个大）",
                   ylabel="归一化 RMS (0~1)"),
}


def make_figure(w_in, h_in):
    """建 Figure。新版 matplotlib 用 constrained 布局，省掉每次重绘调 tight_layout。"""
    try:
        return Figure(figsize=(w_in, h_in), dpi=100, layout="constrained")
    except TypeError:                       # matplotlib < 3.5
        fig = Figure(figsize=(w_in, h_in), dpi=100)
        fig.set_layout_engine("none")
        return fig


def autoscale_y(ax, y):
    """按数据范围自动定 y 轴，留 12% 边距；数据是常数时给一个最小跨度。"""
    finite = y[np.isfinite(y)]
    if finite.size == 0:
        return
    lo = float(np.min(finite))
    hi = float(np.max(finite))
    if hi - lo < 1e-9:
        pad = max(abs(hi) * 0.1, 1e-3)
    else:
        pad = (hi - lo) * 0.12
    ax.set_ylim(lo - pad, hi + pad)


class ZoomWindow:
    """双击某个图弹出来的放大窗口，跟着主窗口一起刷新。"""

    def __init__(self, master, app, key):
        self.app = app
        self.key = key
        meta = CHART_META[key]

        self.win = tk.Toplevel(master)
        self.win.title("放大视图 · " + meta["title"])
        self.win.geometry("1180x560")
        self.win.protocol("WM_DELETE_WINDOW", self.close)

        self.fig = make_figure(11.4, 4.8)
        self.ax = self.fig.add_subplot(111)
        box = ttk.Frame(self.win)
        box.pack(fill="both", expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=box)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, box).update()

        self.line, = self.ax.plot([], [], lw=1.6, color=meta["color"])
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(meta["title"], fontsize=10)
        self.ax.set_ylabel(meta["ylabel"], fontsize=9)
        self.ax.set_xlabel("时间 (s)", fontsize=9)
        self.readout = self.ax.text(
            0.995, 0.95, "—", transform=self.ax.transAxes,
            ha="right", va="top", fontsize=15, family="Consolas",
            bbox=dict(boxstyle="round,pad=0.3", fc="white",
                      ec=meta["color"], alpha=0.9))

    def close(self):
        self.app.popups.pop(self.key, None)
        try:
            self.win.destroy()
        except Exception:
            pass

    def update(self, t, y):
        if not self.win.winfo_exists():
            return
        self.line.set_data(t, y)
        autoscale_y(self.ax, y)
        if len(t):
            self.ax.set_xlim(t[0], max(t[-1], t[0] + 1.0))
            meta = CHART_META[self.key]
            self.readout.set_text(meta["fmt"] % y[-1])
            self.ax.set_title(meta["title"], fontsize=10)
        self.canvas.draw_idle()


class SerialMonitorApp:

    def __init__(self, root):
        self.root = root
        self.root.title("STM32 功率计 · 实时监测（分图版）")
        self.root.geometry("1400x1010")
        self.root.minsize(1120, 760)

        self.ser = None
        self.fs = DEFAULT_FS
        self.vmid = DEFAULT_VMID
        self.vdda = DEFAULT_VDDA

        # 数据缓冲：每列一个 deque，超过长度自动丢最老的
        self.buf = {name: deque(maxlen=HISTORY_ROWS) for name in COLUMNS}
        self.latest = None          # 最新一行的解析结果
        self.row_count = 0          # 收到多少行有效数据
        self.drop_count = 0         # 解析失败/丢包计数
        self.last_rx_time = 0.0

        # 接收侧状态：字节先进 _rx，只按 \n 切行，绝不在主线程里等串口
        self._rx = bytearray()      # 还没凑成完整一行的残留
        self._junk_bytes = 0        # 自上次换行以来累计的字节数（用来判断波特率错）
        self._got_newline = False   # 这次连接里见过换行没有
        self._conn_time = 0.0       # 本次连接的时刻
        self._warned = False        # 同一次连接只弹一次警告
        self._dropped_lines = 0     # 因为单次处理上限被丢掉的行数

        # 放大窗口表：key -> ZoomWindow
        self.popups = {}

        # 画图对象
        self.axes = {}
        self.lines = {}
        self.ax_text = {}
        # 时间轴基准：_n_base 是"时间零点"那一行的 N。
        # 板子的 N 是从上电开始数的绝对计数，它不知道上位机清空过，
        # 所以绝不能直接拿 N/fs 当时间——那样清空完曲线又从 6.4 秒开始。
        # 现在统一成 t = (N - _n_base) / fs，三种情况都从 0 开始：
        #   ① 刚连接 / 刚清空 → _n_base = None，下一行数据来了定基准
        #   ② 板子复位（N 突然变小）→ 数据作废，基准重设
        self._n_base = None
        self._last_n = None         # 上一行的 N，用来发现板子复位

        self.build_ui()
        self.refresh_ports()
        self.root.after(PLOT_INTERVAL_MS, self.tick_plot)
        self.root.after(POLL_INTERVAL_MS, self.tick_poll)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------ 界面搭建 ----
    def build_ui(self):
        pad = {"padx": 8, "pady": 4}

        # ---------- 顶部：串口连接 ----------
        row = ttk.Frame(self.root)
        row.pack(fill="x", **pad)

        ttk.Label(row, text="串口").pack(side="left")
        self.cb_port = ttk.Combobox(row, state="readonly", width=42)
        self.cb_port.pack(side="left", padx=(4, 6))

        ttk.Button(row, text="刷新", width=6,
                   command=self.refresh_ports).pack(side="left", padx=2)

        ttk.Label(row, text="波特率").pack(side="left", padx=(12, 0))
        self.cb_baud = ttk.Combobox(row, state="readonly", width=10, values=BAUD_CHOICES)
        self.cb_baud.set(DEFAULT_BAUD)
        self.cb_baud.pack(side="left", padx=4)

        self.btn_conn = ttk.Button(row, text="连接", width=10, command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=8)

        ttk.Button(row, text="清空", width=8, command=self.clear_data).pack(side="left", padx=2)
        ttk.Button(row, text="导出 CSV", width=10,
                   command=self.export_csv).pack(side="left", padx=2)

        # ---------- 参数行 ----------
        row2 = ttk.Frame(self.root)
        row2.pack(fill="x", **pad)

        ttk.Label(row2, text="采样率 (Hz)").pack(side="left")
        self.ent_fs = ttk.Entry(row2, width=8, justify="right")
        self.ent_fs.insert(0, "%.0f" % DEFAULT_FS)
        self.ent_fs.pack(side="left", padx=(4, 12))
        self.ent_fs.bind("<Return>", lambda e: self.read_params())

        ttk.Label(row2, text="ADC 中心 (V)").pack(side="left")
        self.ent_vmid = ttk.Entry(row2, width=6, justify="right")
        self.ent_vmid.insert(0, "%.2f" % DEFAULT_VMID)
        self.ent_vmid.pack(side="left", padx=(4, 12))
        self.ent_vmid.bind("<Return>", lambda e: self.read_params())

        ttk.Label(row2, text="DAC 满量程 (V)").pack(side="left")
        self.ent_vdda = ttk.Entry(row2, width=6, justify="right")
        self.ent_vdda.insert(0, "%.2f" % DEFAULT_VDDA)
        self.ent_vdda.pack(side="left", padx=(4, 12))
        self.ent_vdda.bind("<Return>", lambda e: self.read_params())

        self.auto_x = tk.BooleanVar(value=True)
        ttk.Checkbutton(row2, text="时间轴自动跟随", variable=self.auto_x).pack(
            side="left", padx=(10, 4))

        ttk.Label(row2, text="（改完按回车生效 · 双击任一图可放大）",
                  foreground="#666").pack(side="left")

        # ---------- 实时读数 ----------
        box_val = ttk.LabelFrame(self.root, text="实时数据")
        box_val.pack(fill="x", **pad)

        grid = ttk.Frame(box_val)
        grid.pack(fill="x", padx=8, pady=6)

        # 每个指标一个大字显示。最后三个是从归一化值换算出来的物理量。
        self.lbl_value = {}
        items = [
            ("N",      "点数 N",        "0",         0, 0),
            ("t",      "时间 (s)",      "0.000",     0, 1),
            ("E",      "能量 E (Σx²)",  "0.000000",  0, 2),
            ("Pwin",   "Pwin",          "0.000000",  1, 0),
            ("Pexp",   "Pexp",          "0.000000",  1, 1),
            ("Pall",   "Pall",          "0.000000",  1, 2),
            ("RMSwin", "RMSwin",        "0.0000",    2, 0),
            ("RMSexp", "RMSexp",        "0.0000",    2, 1),
            ("vrms",   "输入 Vrms",     "0.0000 V",  2, 2),
            ("vdac",   "PA4 示波器读数", "0.0000 V",  3, 0),
            ("pwat",   "物理功率 (1Ω)",  "0.0000 W",  3, 1),
            ("note",   "换算",          "×1.65 / ×3.3 / ×2.72", 3, 2),
        ]
        for key, text, default, r, c in items:
            cell = ttk.Frame(grid)
            cell.grid(row=r, column=c, sticky="we", padx=6, pady=3)
            ttk.Label(cell, text=text, foreground="#666").pack(anchor="w")
            lbl = ttk.Label(cell, text=default,
                            font=("Consolas", 15, "bold"), foreground="#1a73e8")
            lbl.pack(anchor="w")
            self.lbl_value[key] = lbl
            grid.columnconfigure(c, weight=1)
        # "换算"那一格不是数值，用灰色小字
        self.lbl_value["note"].configure(font=("Microsoft YaHei UI", 9),
                                         foreground="#888")

        # ---------- 曲线开关 ----------
        row3 = ttk.Frame(self.root)
        row3.pack(fill="x", **pad)

        ttk.Label(row3, text="显示图：").pack(side="left")
        self.var_show = {}
        for key in CHART_ORDER:
            v = tk.BooleanVar(value=True)
            self.var_show[key] = v
            ttk.Checkbutton(row3, text=key, variable=v,
                            command=self.apply_visibility).pack(side="left", padx=4)

        ttk.Button(row3, text="全部放大图（一次开五个）", width=24,
                   command=self.open_all_zoom).pack(side="left", padx=(18, 2))

        # ---------- 图：五个独立坐标系，共享时间轴 ----------
        self.fig = make_figure(13.4, 8.2)
        first = None
        for i, key in enumerate(CHART_ORDER):
            ax = self.fig.add_subplot(len(CHART_ORDER), 1, i + 1, sharex=first)
            if first is None:
                first = ax
            meta = CHART_META[key]
            ax.grid(True, alpha=0.3)
            ax.set_ylabel(meta["ylabel"], fontsize=8)
            ax.set_title(meta["title"], fontsize=9, loc="left", pad=2)
            ax.tick_params(labelsize=8)
            if i != len(CHART_ORDER) - 1:
                ax.tick_params(labelbottom=False)
            line, = ax.plot([], [], lw=1.4, color=meta["color"])
            txt = ax.text(0.995, 0.90, "—", transform=ax.transAxes,
                          ha="right", va="top", fontsize=11, family="Consolas",
                          bbox=dict(boxstyle="round,pad=0.22", fc="white",
                                    ec=meta["color"], alpha=0.88))
            self.axes[key] = ax
            self.lines[key] = line
            self.ax_text[key] = txt

        self.axes[CHART_ORDER[-1]].set_xlabel("时间 (s)", fontsize=9)

        box_fig = ttk.Frame(self.root)
        box_fig.pack(fill="both", expand=True, **pad)
        self.canvas = FigureCanvasTkAgg(self.fig, master=box_fig)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, box_fig).update()

        # 双击某个图 → 弹出放大窗口
        self.canvas.mpl_connect("button_press_event", self.on_fig_click)

        # ---------- 状态栏 ----------
        self.lbl_status = ttk.Label(self.root, text="未连接", anchor="w",
                                    relief="sunken", padding=(8, 4))
        self.lbl_status.pack(fill="x", side="bottom")

    # ------------------------------------------------------------ 串口管理 ----
    def refresh_ports(self):
        if list_ports is None:
            self.set_status("没装 pyserial，先执行： python -m pip install pyserial")
            return
        ports = list(list_ports.comports())
        values = ["%s  |  %s" % (p.device, p.description) for p in ports]
        self.cb_port["values"] = values
        if values:
            # 优先选中看起来像 ST-Link 或 CH340 的口
            pick = 0
            for i, p in enumerate(ports):
                desc = (p.description or "").lower()
                if "st-link" in desc or "stmicro" in desc or "ch340" in desc:
                    pick = i
                    break
            self.cb_port.current(pick)
            self.set_status("找到 %d 个串口" % len(values))
        else:
            self.set_status("没有找到串口，检查 USB 线 / 驱动")

    def toggle_conn(self):
        if self.ser is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror("缺少依赖", "没装 pyserial：\n\npython -m pip install pyserial")
            return
        i = self.cb_port.current()
        if i < 0:
            messagebox.showwarning("没选串口", "先点【刷新】再选一个串口。")
            return

        port = self.cb_port.get().split("|")[0].strip()
        baud = int(self.cb_baud.get())

        try:
            # timeout=0：读串口完全非阻塞，读到多少算多少，绝不在主线程里等
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0)
            self.ser.reset_input_buffer()
        except Exception as exc:
            self.ser = None
            messagebox.showerror("打开串口失败",
                                 "端口：%s\n波特率：%d\n\n%s\n\n"
                                 "常见原因：串口被别的程序占用（串口助手没关）、"
                                 "USB 线只是充电线、驱动没装。" % (port, baud, exc))
            return

        self.read_params()
        self.clear_data()
        self._rx = bytearray()
        self._junk_bytes = 0
        self._got_newline = False
        self._warned = False
        self._conn_time = time.time()
        self.btn_conn.configure(text="断开")
        self.set_status("已连接 %s @ %d，等待数据…" % (port, baud))

    def disconnect(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.btn_conn.configure(text="连接")
        self.set_status("已断开")

    # ------------------------------------------------------------ 读参数 ----
    def read_params(self):
        try:
            fs = float(self.ent_fs.get())
            if fs <= 0:
                raise ValueError
            self.fs = fs
        except ValueError:
            self.ent_fs.delete(0, "end")
            self.ent_fs.insert(0, "%.0f" % self.fs)
        try:
            self.vmid = float(self.ent_vmid.get())
        except ValueError:
            self.vmid = DEFAULT_VMID
        try:
            self.vdda = float(self.ent_vdda.get())
        except ValueError:
            self.vdda = DEFAULT_VDDA

    # ------------------------------------------------------------ 轮询串口 ----
    def tick_poll(self):
        if self.ser is not None:
            try:
                n = self.ser.in_waiting
                if n:
                    self._feed(self.ser.read(min(n, MAX_BYTES_PER_POLL)))
                self._check_silence()
            except Exception as exc:
                self.set_status("串口读失败：%s" % exc)
                self.disconnect()
        self.root.after(POLL_INTERVAL_MS, self.tick_poll)

    def _feed(self, chunk):
        """把新到的字节接进缓冲。只按 \\n 切行，绝不在主线程里等串口。"""
        self._rx += chunk

        if b"\n" in self._rx:
            parts = self._rx.split(b"\n")
            self._rx = bytearray(parts.pop()[-MAX_LINE_LEN:])   # 最后一段是半行
            self._junk_bytes = 0
            self._got_newline = True
            if len(parts) > MAX_LINES_PER_POLL:
                # 超出单次处理上限的行要记账，否则状态栏会"看起来一条没丢"
                self._dropped_lines += len(parts) - MAX_LINES_PER_POLL
                parts = parts[-MAX_LINES_PER_POLL:]
            for raw in parts:
                self.handle_line(raw.decode("ascii", errors="ignore").strip())
            self.update_value_labels()      # 一 tick 只刷一次界面，不再每行都刷
            return

        # 一直没有换行：普通串口数据不可能这样，只能是参数/接线不对
        self._junk_bytes += len(chunk)
        if len(self._rx) > MAX_LINE_LEN:
            self._rx = bytearray(self._rx[-MAX_LINE_LEN:])
        if self._junk_bytes > NO_NEWLINE_LIMIT:
            self._reject_stream()

    def _reject_stream(self):
        """收到一堆字节却一个换行都没有 —— 直接说清楚，别让界面默默冻住。"""
        if self._warned:
            return
        self._warned = True
        junk = self._junk_bytes
        self.disconnect()
        messagebox.showerror(
            "收到的不是文本",
            "已经收到 %d 字节，但一个换行符都没有。\n\n"
            "这不是脚本的问题，是串口参数对不上：\n"
            "  · 波特率不匹配（最常见 —— 比如固件 HSE_VALUE 配错，"
            "实际跑成了约 38400）\n"
            "  · TX/RX 没交叉接，或者没共地\n"
            "  · 板子里烧的不是这个工程\n\n"
            "换个波特率再连一次试试。" % junk)
        self.set_status("已断开：收到的数据里没有换行，串口参数不匹配")

    def _check_silence(self):
        """连上了却一个字节都没有 —— 提醒查接线，别干等。"""
        if self._got_newline or self.row_count or self._warned:
            return
        if time.time() - self._conn_time > RX_SILENCE_SEC:
            self._warned = True
            self.disconnect()
            messagebox.showwarning(
                "没有数据",
                "连上 %.0f 秒，一个字节都没收到。\n\n"
                "检查：板子有没有上电、TX/RX 是不是交叉接了、GND 有没有共地、"
                "板子里跑的固件是否在发数据。" % RX_SILENCE_SEC)
            self.set_status("已断开：%d 秒内没收到任何数据" % RX_SILENCE_SEC)

    def handle_line(self, line):
        if not line:
            return

        # 表头直接跳过
        if line.startswith("N,"):
            self.set_status("收到表头，串口通了，等数据…")
            return

        parts = line.split(",")
        if len(parts) != len(COLUMNS):
            self.drop_count += 1
            return

        try:
            vals = [float(p) for p in parts]
        except ValueError:
            self.drop_count += 1
            return

        # 时间零点：清空后 / 板子复位后，时间轴重新从 0 开始算，
        # 而不是接着板子的绝对 N 一路涨下去。
        n_now = vals[0]
        if self._n_base is None:
            self._n_base = n_now
        elif self._last_n is not None and n_now < self._last_n:
            # 板子复位：旧数据全部作废，时间重新从 0 起
            self._n_base = n_now
            self.buf = {name: deque(maxlen=HISTORY_ROWS) for name in COLUMNS}
        self._last_n = n_now

        for name, v in zip(COLUMNS, vals):
            self.buf[name].append(v)

        self.latest = dict(zip(COLUMNS, vals))
        self.row_count += 1
        self.last_rx_time = time.time()

    def update_value_labels(self):
        if self.latest is None:
            return
        d = self.latest
        n = int(d["N"])
        t = (n - self._n_base) / self.fs if self._n_base is not None else 0.0
        vrms = d["RMSwin"] * self.vmid           # 归一化 RMS × ADC 中心 = 输入 Vrms
        vdac = d["RMSwin"] * self.vdda           # 板子 DAC 实际会输出的电压
        pwat = d["Pwin"] * self.vmid ** 2        # 归一化功率 × 1.65² = V²（1Ω 下即 W）

        self.lbl_value["N"].configure(text="%d" % n)
        self.lbl_value["t"].configure(text="%.3f" % t)
        self.lbl_value["E"].configure(text="%.6f" % d["E"])
        self.lbl_value["Pwin"].configure(text="%.6f" % d["Pwin"])
        self.lbl_value["Pexp"].configure(text="%.6f" % d["Pexp"])
        self.lbl_value["Pall"].configure(text="%.6f" % d["Pall"])
        self.lbl_value["RMSwin"].configure(text="%.4f" % d["RMSwin"])
        self.lbl_value["RMSexp"].configure(text="%.4f" % d["RMSexp"])
        self.lbl_value["vrms"].configure(text="%.4f V" % vrms)
        self.lbl_value["vdac"].configure(text="%.4f V" % vdac)
        self.lbl_value["pwat"].configure(text="%.4f W" % pwat)

    # ------------------------------------------------------------ 画图 ----
    def tick_plot(self):
        self.redraw()
        for z in list(self.popups.values()):
            try:
                z.update(*self._series(z.key))
            except Exception:
                pass
        self.root.after(PLOT_INTERVAL_MS, self.tick_plot)

    def _series(self, key):
        """取出某个量的 (时间轴, 数值) 数组。时间轴从本次记录的起点算起。"""
        base = 0.0 if self._n_base is None else float(self._n_base)
        t = (np.array(self.buf["N"], dtype=float) - base) / self.fs
        y = np.array(self.buf[key], dtype=float)
        return t, y

    def apply_visibility(self):
        for key in CHART_ORDER:
            self.axes[key].set_visible(self.var_show[key].get())
        self.redraw()

    def redraw(self):
        n = len(self.buf["N"])
        if n < 2:
            for key in CHART_ORDER:
                self.lines[key].set_data([], [])
                self.ax_text[key].set_text("—")
            self.canvas.draw_idle()
            return

        for key in CHART_ORDER:
            t, y = self._series(key)
            ax = self.axes[key]
            self.lines[key].set_data(t, y)
            autoscale_y(ax, y)
            if self.auto_x.get():
                ax.set_xlim(t[0], max(t[-1], t[0] + 1.0))
            meta = CHART_META[key]
            self.ax_text[key].set_text(meta["fmt"] % y[-1] + meta["unit"])

        self.canvas.draw_idle()

    # -------------------------------------------------- 放大窗口 ----
    def on_fig_click(self, event):
        if not event.dblclick or event.inaxes is None:
            return
        for key, ax in self.axes.items():
            if ax is event.inaxes:
                self.open_zoom(key)
                return

    def open_zoom(self, key):
        existing = self.popups.get(key)
        if existing is not None and existing.win.winfo_exists():
            existing.win.lift()
            existing.win.focus_set()
            return
        self.popups[key] = ZoomWindow(self.root, self, key)

    def open_all_zoom(self):
        for key in CHART_ORDER:
            self.open_zoom(key)

    # ------------------------------------------------------------ 其他操作 ----
    def clear_data(self):
        for d in self.buf.values():
            d.clear()
        self.latest = None
        self.row_count = 0
        self.drop_count = 0
        self._dropped_lines = 0
        self._last_n = None
        self._n_base = None          # 下一个数据点成为新的时间零点
        self._rx = bytearray()
        self._junk_bytes = 0
        # 恢复默认显示值（注意带单位的格子要保留单位，别一律写成 "0"）
        defaults = {"N": "0", "t": "0.000", "E": "0.000000",
                    "Pwin": "0.000000", "Pexp": "0.000000", "Pall": "0.000000",
                    "RMSwin": "0.0000", "RMSexp": "0.0000",
                    "vrms": "0.0000 V", "vdac": "0.0000 V", "pwat": "0.0000 W"}
        for k, v in defaults.items():
            if k in self.lbl_value:
                self.lbl_value[k].configure(text=v)
        for key in CHART_ORDER:
            self.lines[key].set_data([], [])
            self.ax_text[key].set_text("—")
        self.canvas.draw_idle()
        self.set_status("已清空")

    def export_csv(self):
        if len(self.buf["N"]) == 0:
            messagebox.showwarning("没有数据", "先连接串口收一点数据。")
            return
        path = filedialog.asksaveasfilename(
            title="导出 CSV", defaultextension=".csv",
            initialfile="power_log.csv",
            filetypes=[("CSV 文件", "*.csv")])
        if not path:
            return
        try:
            base = 0.0 if self._n_base is None else float(self._n_base)
            with open(path, "w", newline="", encoding="utf-8-sig") as fh:
                fh.write("t_s,vrms_v,vdac_v,p_watt," + ",".join(COLUMNS) + "\n")
                rows = zip(*(self.buf[c] for c in COLUMNS))
                for i, row in enumerate(rows):
                    t = (row[0] - base) / self.fs
                    vrms = row[5] * self.vmid
                    fh.write("%.6f,%.6f,%.6f,%.6f," % (t, vrms, row[5] * self.vdda,
                                                       row[2] * self.vmid ** 2))
                    fh.write(",".join("%.9g" % v for v in row))
                    fh.write("\n")
            self.set_status("已导出 %d 行到 %s" % (len(self.buf["N"]), path))
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))

    def set_status(self, text):
        if getattr(self, "lbl_status", None) is not None:
            extra = ""
            if self.row_count:
                extra = "　|　已收 %d 行" % self.row_count
                drop = self.drop_count + self._dropped_lines
                if drop:
                    extra += "，丢 %d 行" % drop
            self.lbl_status.configure(text=text + extra)

    def on_close(self):
        for z in list(self.popups.values()):
            try:
                z.win.destroy()
            except Exception:
                pass
        self.popups.clear()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        f = tkfont.nametofont("TkDefaultFont")
        f.configure(family="Microsoft YaHei UI", size=10)
    except Exception:
        pass
    SerialMonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
