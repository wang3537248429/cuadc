from __future__ import annotations

import argparse
import json
import queue
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .model import (
    COMPANION_COMPONENT_ID,
    MISSION_STAGES_ZH,
    STATUS_NAMES_ZH,
    TASK1_TARGETS,
    ReportSnapshot,
    mission_stage_zh,
    mission_mode_name_zh,
    target_name_zh,
)
from .tlog_reader import DEFAULT_TLOG_ROOT, TlogFollower, newest_tlog, scan_tlog


def preferred_ui_font() -> str:
    if sys.platform == "win32":
        return "Microsoft YaHei UI"
    if sys.platform == "darwin":
        return "PingFang SC"
    return "Noto Sans CJK SC"


class MonitorWindow:
    def __init__(
        self,
        root: tk.Tk,
        tlog_root: Path,
        component_id: int = COMPANION_COMPONENT_ID,
        offline_seconds: float = 5.0,
    ) -> None:
        self.root = root
        self.offline_seconds = offline_seconds
        self.snapshot = ReportSnapshot(component_id=component_id)
        self.messages: "queue.Queue[tuple[str, object]]" = queue.Queue(maxsize=30)
        self.follower = TlogFollower(self.messages, tlog_root, component_id)
        self.last_event_key: tuple[float, str] | None = None

        self.root.title("CUADC 靶标回传监控")
        screen_width = max(1024, self.root.winfo_screenwidth())
        screen_height = max(720, self.root.winfo_screenheight())
        window_width = min(1180, screen_width - 40)
        window_height = min(900, screen_height - 80)
        self.root.geometry(f"{window_width}x{window_height}")
        self.root.minsize(min(980, window_width), min(680, window_height))
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._configure_style()
        self._build_ui()
        self.follower.start()
        self.root.after(150, self._drain_messages)
        self.root.after(500, self._refresh_freshness)

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        font = preferred_ui_font()
        self.root.option_add("*Font", (font, 11))
        style.configure("Header.TLabel", font=(font, 20, "bold"))
        style.configure("Big.TLabel", font=(font, 27, "bold"))
        style.configure("Value.TLabel", font=(font, 12, "bold"))
        style.configure("Safe.TLabel", foreground="#137333", font=(font, 12, "bold"))
        style.configure("Danger.TLabel", foreground="#b3261e", font=(font, 12, "bold"))
        style.configure("Treeview", rowheight=29)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=14)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)
        outer.columnconfigure(0, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="CUADC 靶标回传监控", style="Header.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.connection_badge = tk.Label(
            header,
            text="等待组件 191",
            bg="#9aa0a6",
            fg="white",
            padx=12,
            pady=6,
            font=(preferred_ui_font(), 11, "bold"),
        )
        self.connection_badge.grid(row=0, column=2, sticky="e")
        self.mission_badge = tk.Label(
            header,
            text="航线等待回读",
            bg="#9aa0a6",
            fg="white",
            padx=12,
            pady=6,
            font=(preferred_ui_font(), 11, "bold"),
        )
        self.mission_badge.grid(row=0, column=1, sticky="e", padx=(8, 8))

        notice = ttk.Label(
            outer,
            text="只读取 Mission Planner 日志 · 同屏显示飞控与树莓派 MAVLink 数据 · 不打开串口 · 不占用 UDP 端口",
            foreground="#5f6368",
        )
        notice.grid(row=1, column=0, sticky="w", pady=(0, 10))

        notebook = ttk.Notebook(outer)
        notebook.grid(row=2, column=0, sticky="nsew")
        live = ttk.Frame(notebook, padding=10)
        names = ttk.Frame(notebook, padding=10)
        notebook.add(live, text="实时监控")
        notebook.add(names, text="中文靶标对照")
        self._build_live_tab(live)
        self._build_names_tab(names)

        footer = ttk.Frame(outer)
        footer.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        footer.columnconfigure(0, weight=1)
        self.source_var = tk.StringVar(value="正在查找 Mission Planner 日志…")
        ttk.Label(footer, textvariable=self.source_var, foreground="#5f6368").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Button(footer, text="复制中文靶标名", command=self.copy_target_name).grid(
            row=0, column=1, padx=6
        )
        ttk.Button(footer, text="导出本组中文 JSON", command=self.export_json).grid(
            row=0, column=2
        )

    def _build_live_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(3, weight=1)

        detected = ttk.LabelFrame(parent, text="当前识别靶标", padding=12)
        detected.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        selected = ttk.LabelFrame(parent, text="扫描结束后的最终选靶", padding=12)
        selected.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))

        self.detected_name = tk.StringVar(value="暂未识别")
        self.detected_detail = tk.StringVar(value="编号 --  ·  稳定目标 0  ·  置信度 --")
        self.detected_position = tk.StringVar(value="HOME 坐标：北 0.0 m / 东 0.0 m")
        ttk.Label(detected, textvariable=self.detected_name, style="Big.TLabel").pack(anchor="w")
        ttk.Label(detected, textvariable=self.detected_detail).pack(anchor="w", pady=(8, 3))
        ttk.Label(detected, textvariable=self.detected_position).pack(anchor="w")

        self.selected_name = tk.StringVar(value="尚未选靶")
        self.selected_detail = tk.StringVar(value="编号 --")
        self.selected_position = tk.StringVar(value="HOME 坐标：北 0.0 m / 东 0.0 m")
        ttk.Label(selected, textvariable=self.selected_name, style="Big.TLabel").pack(anchor="w")
        ttk.Label(selected, textvariable=self.selected_detail).pack(anchor="w", pady=(8, 3))
        ttk.Label(selected, textvariable=self.selected_position).pack(anchor="w")

        task = ttk.LabelFrame(parent, text="任务与导航", padding=10)
        task.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        safety = ttk.LabelFrame(parent, text="程序与投水安全状态", padding=10)
        safety.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))

        self.task_vars = self._make_value_grid(
            task,
            ("识别任务", "航线序号", "当前航线阶段", "GPS / 卫星 / HDOP", "高度 / 空速", "YOLO累计推理"),
        )
        self.safety_vars = self._make_value_grid(
            safety,
            (
                "航线通行",
                "树莓派航线模式",
                "机载程序",
                "USB摄像头",
                "主动投水",
                "已发送命令",
                "已观察PWM输出",
                "最后回传",
            ),
        )

        flight = ttk.LabelFrame(parent, text="飞控 MAVLink 实时遥测（COMPID 1）", padding=10)
        flight.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(0, 8))
        self.flight_vars = self._make_two_column_value_grid(
            flight,
            (
                "飞控连接",
                "飞行模式 / 解锁",
                "GPS / 卫星 / HDOP",
                "经纬度",
                "高度 / 速度",
                "姿态 / 航向",
                "电池",
                "任务序号",
                "链路质量",
                "最近飞控消息",
            ),
        )

        events = ttk.LabelFrame(parent, text="树莓派事件（已合并重复消息）", padding=8)
        events.grid(row=3, column=0, columnspan=2, sticky="nsew")
        events.rowconfigure(0, weight=1)
        events.columnconfigure(0, weight=1)
        self.event_tree = ttk.Treeview(
            events,
            columns=("time", "level", "message"),
            show="headings",
            height=7,
        )
        self.event_tree.heading("time", text="时间")
        self.event_tree.heading("level", text="级别")
        self.event_tree.heading("message", text="中文消息")
        self.event_tree.column("time", width=90, anchor="center", stretch=False)
        self.event_tree.column("level", width=80, anchor="center", stretch=False)
        self.event_tree.column("message", width=760)
        scrollbar = ttk.Scrollbar(events, orient="vertical", command=self.event_tree.yview)
        self.event_tree.configure(yscrollcommand=scrollbar.set)
        self.event_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

    def _make_value_grid(self, parent: ttk.Frame, labels: tuple[str, ...]) -> dict[str, tk.StringVar]:
        variables: dict[str, tk.StringVar] = {}
        parent.columnconfigure(1, weight=1)
        for row, label in enumerate(labels):
            ttk.Label(parent, text=f"{label}：", foreground="#5f6368").grid(
                row=row, column=0, sticky="w", padx=(0, 12), pady=2
            )
            variable = tk.StringVar(value="--")
            ttk.Label(parent, textvariable=variable, style="Value.TLabel").grid(
                row=row, column=1, sticky="w", pady=2
            )
            variables[label] = variable
        return variables

    def _make_two_column_value_grid(
        self, parent: ttk.Frame, labels: tuple[str, ...]
    ) -> dict[str, tk.StringVar]:
        variables: dict[str, tk.StringVar] = {}
        parent.columnconfigure(1, weight=1)
        parent.columnconfigure(3, weight=1)
        split = (len(labels) + 1) // 2
        for index, label in enumerate(labels):
            column_group = 0 if index < split else 1
            row = index if column_group == 0 else index - split
            label_column = column_group * 2
            ttk.Label(parent, text=f"{label}：", foreground="#5f6368").grid(
                row=row, column=label_column, sticky="w", padx=(0, 10), pady=2
            )
            variable = tk.StringVar(value="--")
            ttk.Label(parent, textvariable=variable, style="Value.TLabel").grid(
                row=row,
                column=label_column + 1,
                sticky="w",
                padx=(0, 22) if column_group == 0 else 0,
                pady=2,
            )
            variables[label] = variable
        return variables

    def _build_names_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        ttk.Label(
            parent,
            text="任务一的回传数值会自动转成下列中文名称；任务二显示为“XX 号数字靶”。",
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))
        tree = ttk.Treeview(
            parent,
            columns=("value", "zh", "model", "fill"),
            show="headings",
        )
        tree.heading("value", text="回传编号")
        tree.heading("zh", text="中文靶标名")
        tree.heading("model", text="模型原始类别")
        tree.heading("fill", text="比赛填写内容")
        tree.column("value", width=110, anchor="center", stretch=False)
        tree.column("zh", width=180, anchor="center")
        tree.column("model", width=230, anchor="center")
        tree.column("fill", width=240, anchor="center")
        for value, (label, model_name) in TASK1_TARGETS.items():
            tree.insert("", "end", values=(value, label, model_name, label))
        tree.grid(row=1, column=0, sticky="nsew")

        stage_frame = ttk.LabelFrame(parent, text="动态航线说明", padding=8)
        stage_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        stage_text = (
            "航点总数和绝对序号不固定。界面根据树莓派回传的 9101、9102、投水槽和 "
            "NAV_LAND 实际序号解释当前阶段；普通航线显示为普通任务点，树莓派不干预。"
        )
        ttk.Label(stage_frame, text=stage_text, wraplength=1080, justify="left").pack(anchor="w")

    def _drain_messages(self) -> None:
        try:
            while True:
                kind, payload = self.messages.get_nowait()
                if kind == "snapshot" and isinstance(payload, ReportSnapshot):
                    self.snapshot = payload
                    self._render_snapshot()
                elif kind in {"status", "error"}:
                    self.source_var.set(str(payload))
        except queue.Empty:
            pass
        self.root.after(150, self._drain_messages)

    def _refresh_freshness(self) -> None:
        self._render_connection()
        self.root.after(500, self._refresh_freshness)

    def _render_connection(self) -> None:
        if self.snapshot.is_online(timeout_s=self.offline_seconds):
            self.connection_badge.configure(text="树莓派组件 191 在线", bg="#188038")
        elif self.snapshot.latest_at:
            age = max(0, int(time.time() - self.snapshot.latest_at))
            self.connection_badge.configure(text=f"组件 191 离线 {age}秒", bg="#b3261e")
        else:
            self.connection_badge.configure(text="等待组件 191", bg="#9aa0a6")

    def _render_snapshot(self) -> None:
        v = self.snapshot.values
        task = int(v.get("CU_TASK", 1))
        detected = int(v.get("CU_DETVAL", -1))
        selected = int(v.get("CU_SELVAL", -1))
        confidence = float(v.get("CU_DETCF", -1.0))
        self.detected_name.set(target_name_zh(task, detected))
        self.detected_detail.set(
            f"编号 {detected if detected >= 0 else '--'}  ·  稳定目标 {int(v.get('CU_DETCNT', 0))}  ·  "
            + (f"置信度 {confidence:.1%}" if confidence >= 0 else "置信度 --")
        )
        self.detected_position.set(
            f"HOME 坐标：北 {float(v.get('CU_DETN', 0.0)):.1f} m / 东 {float(v.get('CU_DETE', 0.0)):.1f} m"
        )
        self.selected_name.set(target_name_zh(task, selected) if selected >= 0 else "尚未选靶")
        self.selected_detail.set(f"编号 {selected if selected >= 0 else '--'}")
        self.selected_position.set(
            f"HOME 坐标：北 {float(v.get('CU_SELN', 0.0)):.1f} m / 东 {float(v.get('CU_SELE', 0.0)):.1f} m"
        )

        seq = int(v.get("CU_MSEQ", 0))
        hdop = float(v.get("CU_HDOP", -1.0))
        self.task_vars["识别任务"].set(f"任务 {task}")
        self.task_vars["航线序号"].set(str(seq))
        self.task_vars["当前航线阶段"].set(mission_stage_zh(seq, v))
        self.task_vars["GPS / 卫星 / HDOP"].set(
            f"Fix {int(v.get('CU_GPSFIX', 0))} / {int(v.get('CU_SATS', 0))} 颗 / "
            + (f"{hdop:.2f}" if hdop >= 0 else "--")
        )
        self.task_vars["高度 / 空速"].set(
            f"{float(v.get('CU_ALT', -1.0)):.1f} m / {float(v.get('CU_AIRSPD', 0.0)):.1f} m/s"
        )
        self.task_vars["YOLO累计推理"].set(f"{int(round(float(v.get('CU_INF', 0.0))))} 次")

        status_code = int(v.get("CU_STATUS", 99))
        mission_passed = bool(int(v.get("CU_MPASS", 0)))
        mission_mode = int(v.get("CU_MMODE", 0))
        self.safety_vars["航线通行"].set(
            "已通过，飞控可执行" if mission_passed else "等待稳定回读 / 未通过"
        )
        self.safety_vars["树莓派航线模式"].set(
            mission_mode_name_zh(mission_mode)
        )
        if mission_passed:
            self.mission_badge.configure(text="航线已通过", bg="#188038")
        elif status_code == 17:
            self.mission_badge.configure(text="航线等待回读", bg="#f29900")
        else:
            self.mission_badge.configure(text="航线未通过", bg="#b3261e")
        self.safety_vars["机载程序"].set(STATUS_NAMES_ZH.get(status_code, "未知状态"))
        camera_online = bool(int(v.get("CU_CAMOK", 0)))
        camera_mode = "自动选择" if int(v.get("CU_CAMAUTO", 0)) else "固定设备"
        self.safety_vars["USB摄像头"].set(
            f"{'在线' if camera_online else '离线'} · {camera_mode} · "
            f"{int(v.get('CU_CAMW', 0))}×{int(v.get('CU_CAMH', 0))} "
            f"@ {int(v.get('CU_CAMFPS', 0))}fps"
        )
        self.safety_vars["主动投水"].set(
            "允许（请核对安全）" if int(v.get("CU_REL_EN", 0)) else "禁止（配置未开启）"
        )
        self.safety_vars["已发送命令"].set("是" if int(v.get("CU_REL_CMD", 0)) else "否")
        self.safety_vars["已观察PWM输出"].set("是" if int(v.get("CU_REL_OBS", 0)) else "否")
        self.safety_vars["最后回传"].set(
            time.strftime("%H:%M:%S", time.localtime(self.snapshot.latest_at))
            if self.snapshot.latest_at
            else "尚未收到"
        )
        f = self.snapshot.flight_values
        flight_online = self.snapshot.is_flight_controller_online(
            timeout_s=self.offline_seconds
        )
        self.flight_vars["飞控连接"].set(
            f"{'在线' if flight_online else '离线'} · SYSID {self.snapshot.flight_system_id or '--'}"
        )
        self.flight_vars["飞行模式 / 解锁"].set(
            f"{f.get('FC_MODE', '未知')} / {'已解锁' if int(f.get('FC_ARMED', 0)) else '未解锁'}"
        )
        fc_hdop = float(f.get("FC_HDOP", -1.0))
        self.flight_vars["GPS / 卫星 / HDOP"].set(
            f"Fix {int(f.get('FC_GPSFIX', 0))} / {int(f.get('FC_SATS', 0))} 颗 / "
            + (f"{fc_hdop:.2f}" if fc_hdop >= 0 else "--")
        )
        latitude = float(f.get("FC_LAT", 0.0))
        longitude = float(f.get("FC_LON", 0.0))
        fc_gps_fix = int(f.get("FC_GPSFIX", 0))
        self.flight_vars["经纬度"].set(
            f"{latitude:.7f}, {longitude:.7f}" if fc_gps_fix >= 2 else "--（GPS未定位）"
        )
        relative_altitude = (
            f"{float(f.get('FC_REL_ALT', 0.0)):.1f} m" if fc_gps_fix >= 3 else "--"
        )
        self.flight_vars["高度 / 速度"].set(
            f"相对 {relative_altitude} · "
            f"空速 {float(f.get('FC_AIRSPD', 0.0)):.1f} / 地速 {float(f.get('FC_GNDSPD', 0.0)):.1f} m/s"
        )
        heading = float(f.get("FC_HEADING", -1.0))
        self.flight_vars["姿态 / 航向"].set(
            f"横滚 {float(f.get('FC_ROLL', 0.0)):.1f}° · "
            f"俯仰 {float(f.get('FC_PITCH', 0.0)):.1f}° · "
            + (f"航向 {heading:.0f}°" if heading >= 0 else "航向 --")
        )
        battery_v = float(f.get("FC_BAT_V", -1.0))
        battery_a = float(f.get("FC_BAT_A", -1.0))
        battery_remaining = int(f.get("FC_BAT_REMAIN", -1))
        battery_text = f"{battery_v:.2f} V" if battery_v >= 0 else "-- V"
        if battery_a >= 0:
            battery_text += f" / {battery_a:.1f} A"
        if battery_remaining >= 0:
            battery_text += f" / {battery_remaining}%"
        self.flight_vars["电池"].set(battery_text)
        total = int(f.get("FC_MTOTAL", -1))
        total_text = str(total) if total >= 0 else "--"
        self.flight_vars["任务序号"].set(
            f"{int(f.get('FC_MSEQ', -1))} / {total_text} · "
            f"{mission_stage_zh(int(f.get('FC_MSEQ', -1)), v)}"
        )
        drop_rate = float(f.get("FC_DROP_RATE", -1.0))
        rssi = int(f.get("FC_RADIO_RSSI", -1))
        link_parts = [f"通信丢包 {drop_rate:.2f}%" if drop_rate >= 0 else "通信丢包 --"]
        if rssi >= 0:
            link_parts.append(f"RSSI {rssi}/255")
        self.flight_vars["链路质量"].set(" · ".join(link_parts))
        self.flight_vars["最近飞控消息"].set(
            self.snapshot.flight_events[-1].text_zh
            if self.snapshot.flight_events
            else "无"
        )
        self.source_var.set(self.snapshot.source_path or "尚未找到 Mission Planner 日志")
        self._render_connection()
        self._render_events()

    def _render_events(self) -> None:
        events = list(self.snapshot.events)
        event_key = (
            (events[-1].received_at, events[-1].raw_text)
            if events
            else None
        )
        if event_key == self.last_event_key:
            return
        self.last_event_key = event_key
        for item in self.event_tree.get_children():
            self.event_tree.delete(item)
        levels = {0: "紧急", 1: "警报", 2: "严重", 3: "错误", 4: "警告", 5: "通知", 6: "信息", 7: "调试"}
        for event in reversed(events[-50:]):
            self.event_tree.insert(
                "",
                "end",
                values=(
                    time.strftime("%H:%M:%S", time.localtime(event.received_at)),
                    levels.get(event.severity, str(event.severity)),
                    event.text_zh,
                ),
            )

    def copy_target_name(self) -> None:
        value = self.snapshot.selected_value
        if value < 0:
            value = self.snapshot.detected_value
        text = target_name_zh(self.snapshot.task, value)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.source_var.set(f"已复制：{text}")

    def export_json(self) -> None:
        suggested = time.strftime("CUADC靶标信息_%Y%m%d_%H%M%S.json")
        path = filedialog.asksaveasfilename(
            parent=self.root,
            title="导出本组中文靶标信息",
            initialfile=suggested,
            defaultextension=".json",
            filetypes=(("JSON 文件", "*.json"),),
        )
        if not path:
            return
        try:
            Path(path).write_text(
                json.dumps(
                    self.snapshot.grouped_zh(timeout_s=self.offline_seconds),
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            messagebox.showerror("导出失败", str(exc), parent=self.root)
            return
        self.source_var.set(f"已导出：{path}")

    def close(self) -> None:
        self.follower.stop()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CUADC 组件191中文靶标回传监控")
    parser.add_argument("--tlog-root", type=Path, default=DEFAULT_TLOG_ROOT)
    parser.add_argument("--component", type=int, default=COMPANION_COMPONENT_ID)
    parser.add_argument("--offline-seconds", type=float, default=5.0)
    parser.add_argument(
        "--snapshot-json",
        action="store_true",
        help="读取最新日志并在终端输出一组中文JSON后退出",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="与 --snapshot-json 同用，将 JSON 写入文件（便于 Windows 版自检）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.offline_seconds <= 0:
        raise SystemExit("--offline-seconds 必须大于 0")
    if args.output is not None and not args.snapshot_json:
        raise SystemExit("--output 只能与 --snapshot-json 同时使用")
    if args.snapshot_json:
        path = newest_tlog(args.tlog_root)
        if path is None:
            raise SystemExit(f"未找到 .tlog：{args.tlog_root}")
        snapshot = scan_tlog(path, args.component)
        rendered = json.dumps(
            snapshot.grouped_zh(timeout_s=args.offline_seconds),
            ensure_ascii=False,
            indent=2,
        )
        if args.output is None:
            print(rendered)
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n", encoding="utf-8")
        return 0
    root = tk.Tk()
    MonitorWindow(root, args.tlog_root, args.component, args.offline_seconds)
    root.mainloop()
    return 0
