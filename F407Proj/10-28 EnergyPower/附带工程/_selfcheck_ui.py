# -*- coding: utf-8 -*-
"""
_selfcheck_ui.py —— serial_monitor.py 的离屏自检
不连板子、不连串口，直接灌模拟数据进解析与绘图链路，验证：
  1) 模块能导入、界面能搭起来（5 个独立图）
  2) handle_line 能正确解析 7 列 CSV
  3) 换算关系（Vrms / PA4 / 物理功率）算得对
  4) redraw / 放大窗口 update 不抛异常
  5) 板子复位（N 变小）时时间轴不错乱
用法： python _selfcheck_ui.py
"""

import importlib.util
import math
import os
import sys
import tempfile

sys.stdout.reconfigure(line_buffering=True)

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "serial_monitor.py")

spec = importlib.util.spec_from_file_location("serial_monitor", TARGET)
sm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sm)

import tkinter as tk

FAIL = []


def check(name, cond, detail=""):
    if cond:
        print("  [PASS] %s %s" % (name, detail))
    else:
        print("  [FAIL] %s %s" % (name, detail))
        FAIL.append(name)


print("=" * 70)
print("1) 建界面")
root = tk.Tk()
root.withdraw()                      # 不真的弹窗口
app = sm.SerialMonitorApp(root)
check("五个图都建好了", len(app.axes) == 5, str(list(app.axes)))
check("坐标轴共享时间轴",
      app.axes["Pwin"].get_shared_x_axes().joined(app.axes["Pwin"], app.axes["RMSexp"]))
check("五个图的曲线颜色: Pwin==RMSwin",
      sm.CHART_META["Pwin"]["color"] == sm.CHART_META["RMSwin"]["color"])
check("五个图的曲线颜色: Pexp==RMSexp",
      sm.CHART_META["Pexp"]["color"] == sm.CHART_META["RMSexp"]["color"])

print("=" * 70)
print("2) 灌 600 行 CSV（复刻 1 kHz / 2 Vpp 正弦在 48 kHz 下的理论值）")
FS = 48000.0
P_THEORY = 0.183655
for i in range(1, 601):
    n = i * 512
    pw = P_THEORY * (1.0 + 0.01 * math.sin(i / 20.0))
    pe = pw * 1.02
    pa = P_THEORY * 0.98
    line = "%.0f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (
        n, n * pw, pw, pe, pa, math.sqrt(pw), math.sqrt(pe))
    app.handle_line(line)

check("收到 600 行", app.row_count == 600, "row_count=%d" % app.row_count)
check("没有解析失败", app.drop_count == 0, "drop_count=%d" % app.drop_count)
check("最新一行在手", app.latest is not None)

print("=" * 70)
print("3) 换算关系（这是这次改动的重点：板子上的数和示波器上的数怎么对）")
app.update_value_labels()

# (a) 理论值链条：Pwin → RMSwin → Vrms → PA4
pw_v = P_THEORY
rms_v = math.sqrt(pw_v)
vrms_v = rms_v * 1.65
check("理论：Pwin 0.183655 → RMSwin 0.428550", abs(rms_v - 0.4285496) < 1e-6,
      "= %.9f" % rms_v)
check("理论：输入 Vrms = 0.707107", abs(vrms_v - 0.707107) < 1e-5,
      "= %.6f V" % vrms_v)
check("理论：PA4 = 2 × Vrms = 1.414214", abs(rms_v * 3.3 - 1.414214) < 1e-5,
      "= %.6f V" % (rms_v * 3.3))

# (b) 程序里实际走的那条换算路径，必须和上面一致
d = app.latest
check("程序换算 Vrms == RMSwin × 1.65",
      abs(float(app.lbl_value["vrms"].cget("text").split()[0]) - d["RMSwin"] * 1.65) < 1e-4,
      "RMSwin=%.4f → %s" % (d["RMSwin"], app.lbl_value["vrms"].cget("text")))
check("程序换算 PA4 == RMSwin × 3.3",
      abs(float(app.lbl_value["vdac"].cget("text").split()[0]) - d["RMSwin"] * 3.3) < 1e-4,
      "= %s" % app.lbl_value["vdac"].cget("text"))
check("程序换算功率 == Pwin × 1.65²",
      abs(float(app.lbl_value["pwat"].cget("text").split()[0]) - d["Pwin"] * 1.65 ** 2) < 1e-4,
      "= %s" % app.lbl_value["pwat"].cget("text"))
check("界面标签写的是 V 单位", app.lbl_value["vdac"].cget("text").endswith("V"),
      repr(app.lbl_value["vdac"].cget("text")))

print("=" * 70)
print("4) 画图链路")
app.redraw()
root.update_idletasks()
check("redraw 无异常", True)
ylim = app.axes["Pwin"].get_ylim()
check("Pwin 的 y 轴自动缩放到数据附近",
      ylim[0] < P_THEORY < ylim[1], "ylim=(%.4f, %.4f)" % ylim)
xlim = app.axes["RMSexp"].get_xlim()
check("x 轴是时间（秒）", xlim[1] > 5.0, "xlim=(%.2f, %.2f)" % xlim)

print("=" * 70)
print("5) 放大窗口")
app.open_zoom("Pexp")
app.open_zoom("Pexp")               # 重复点应该只是 lift，不重复建
check("同名放大窗口只开一个", len(app.popups) == 1, str(list(app.popups)))
z = app.popups["Pexp"]
z.update(*app._series("Pexp"))
root.update_idletasks()
check("放大窗口能独立刷新", len(z.line.get_ydata()) == 600,
      "点数=%d" % len(z.line.get_ydata()))

print("  正在创建其余四个放大窗口…")
for key in sm.CHART_ORDER:
    print("    -> %s" % key)
    app.open_zoom(key)
print("  五个窗口对象都建好了")
check("一键开五个放大图", len(app.popups) == 5, str(sorted(app.popups)))
root.update_idletasks()
print("  界面 idle 任务处理完成")

# 自检里主窗口是 withdraw 的，多个 Toplevel 累积着做完整 update()
# 在无 mainloop 的情况下会卡住（这是测试环境限制，不是产品问题）。
# 所以这里验证完就全部关掉。
for _z in list(app.popups.values()):
    _z.close()
root.update_idletasks()
check("放大窗口能正常关闭", len(app.popups) == 0, str(app.popups))

print("=" * 70)
print("6) 隐藏某个图")
app.var_show["Pall"].set(False)
app.apply_visibility()
root.update_idletasks()
check("Pall 图被隐藏", app.axes["Pall"].get_visible() is False)
app.var_show["Pall"].set(True)
app.apply_visibility()

print("=" * 70)
print("7) 导出 CSV（写进项目目录，不碰 C 盘临时区）")
out = os.path.join(HERE, "_selfcheck_out.csv")
orig = sm.filedialog.asksaveasfilename
sm.filedialog.asksaveasfilename = lambda **kw: out
app.export_csv()
sm.filedialog.asksaveasfilename = orig
check("CSV 文件生成了", os.path.exists(out))
if os.path.exists(out):
    with open(out, encoding="utf-8-sig") as fh:
        head = fh.readline().strip()
        rows = fh.readlines()
    check("表头带换算列", head.startswith("t_s,vrms_v,vdac_v,p_watt,"), head)
    check("行数对", len(rows) == 600, "%d 行" % len(rows))
    first = rows[0].split(",")
    # 列序：0=t_s 1=vrms_v 2=vdac_v 3=p_watt 4=N 5=E 6=Pwin 7=Pexp 8=Pall 9=RMSwin 10=RMSexp
    check("导出的 vrms_v == 同行的 RMSwin × 1.65",
          abs(float(first[1]) - float(first[9]) * 1.65) < 1e-4,
          "vrms_v=%s  RMSwin=%s" % (first[1], first[9]))
    check("导出的 vdac_v == 2 × vrms_v",
          abs(float(first[2]) - 2 * float(first[1])) < 1e-4,
          "vdac_v=%s" % first[2])
    check("导出的 p_watt == 同行的 Pwin × 1.65²",
          abs(float(first[3]) - float(first[6]) * 1.65 ** 2) < 1e-4,
          "p_watt=%s  Pwin=%s" % (first[3], first[6]))

    check("导出的 t_s 从 0 起（不叠加板子的绝对 N）",
          abs(float(first[0])) < 1e-6, "t_s=%s" % first[0])

print("=" * 70)
print("8) 时间轴基准：连接 / 清空 / 复位都必须从 0 开始")
app.update_value_labels()
t_before = float(app.lbl_value["t"].cget("text"))
check("清空前时间不为 0（板子已跑了 6.4 s）", t_before > 6.0,
      "t=%.3f s (N=%d)" % (t_before, app.latest["N"]))

# 清空后板子并不知道，N 继续从 307712 往上数
app.clear_data()
app.handle_line("307712,0.1,0.2,0.2,0.2,0.44,0.44")
app.update_value_labels()
t_after = float(app.lbl_value["t"].cget("text"))
check("清空后时间从 0 重新开始", abs(t_after) < 0.02,
      "t=%.4f s (N=%d，板子仍在往上数)" % (t_after, app.latest["N"]))
t_arr, _ = app._series("N")
check("曲线的横轴也从 0 起", len(t_arr) and abs(t_arr[0]) < 1e-9,
      "t[0]=%.6f" % (t_arr[0] if len(t_arr) else float("nan")))

# 再往后灌几行，时间应该正常增长而不是从 6.4 秒接上
for k in range(1, 5):
    app.handle_line("%d,0.1,0.2,0.2,0.2,0.44,0.44" % (307712 + k * 512))
app.update_value_labels()
t_grow = float(app.lbl_value["t"].cget("text"))
check("之后的时间从新零点正常增长", abs(t_grow - 2048 / FS) < 1e-3,
      "t=%.4f s（应为 4×512/48000=%.4f）" % (t_grow, 2048 / FS))

print("=" * 70)
print("9) 板子复位（N 突然变小）")
app.handle_line("512,0.1,0.2,0.2,0.2,0.44,0.44")
app.update_value_labels()
check("复位后时间重新归零", abs(float(app.lbl_value["t"].cget("text"))) < 0.02,
      "t=%s" % app.lbl_value["t"].cget("text"))
check("复位后旧数据被清掉", len(app.buf["N"]) == 1, "%d 行" % len(app.buf["N"]))

print("=" * 70)
print("10) 清空")
app.clear_data()
root.update_idletasks()
check("数据清空了", len(app.buf["N"]) == 0)
check("带单位的格子保留了单位",
      app.lbl_value["vrms"].cget("text").endswith("V"),
      repr(app.lbl_value["vrms"].cget("text")))

for z in list(app.popups.values()):
    try:
        z.win.destroy()
    except Exception:
        pass
root.destroy()

print("=" * 70)
if FAIL:
    print("自检失败项：", FAIL)
    sys.exit(1)
print("全部自检通过。")
