from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from .config import RUNTIME_DIR


class JsonMtimeCache:
    def __init__(self, path: Path):
        self.path = path
        self.mtime_ns = -1
        self.value: dict = {}

    def load(self) -> tuple[dict, bool]:
        try:
            mtime_ns = self.path.stat().st_mtime_ns
        except FileNotFoundError:
            return self.value, False
        if mtime_ns == self.mtime_ns:
            return self.value, False
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return self.value, False
        self.mtime_ns = mtime_ns
        self.value = value
        return value, True


def run_gui() -> int:
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QPixmap
    from PySide6.QtWidgets import (
        QApplication,
        QHeaderView,
        QLabel,
        QMainWindow,
        QTableWidget,
        QTableWidgetItem,
        QVBoxLayout,
        QWidget,
    )

    class Window(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("CUADC YOLO 侦察识别")
            self.resize(1200, 1000)
            self.vision_cache = JsonMtimeCache(
                RUNTIME_DIR / "vision_state.json"
            )
            self.frame_path = RUNTIME_DIR / "camera_frame.jpg"
            self.frame_mtime_ns = -1
            self.source_pixmap = QPixmap()
            self.last_scaled_size = None

            root = QWidget()
            outer = QVBoxLayout(root)
            outer.setContentsMargins(12, 12, 12, 12)
            outer.setSpacing(10)

            self.banner = QLabel("等待 Gazebo 相机")
            self.banner.setAlignment(Qt.AlignCenter)
            self.banner.setObjectName("banner")
            self.image = QLabel("相机尚未输出")
            self.image.setMinimumSize(900, 380)
            self.image.setAlignment(Qt.AlignCenter)
            self.image.setObjectName("camera")
            self.detail = QLabel("靶标局部预览：等待 YOLO 像素定位")
            self.detail.setAlignment(Qt.AlignCenter)
            self.detail.setFixedHeight(240)
            self.detail_mtime_ns = -1
            # The route observes one physical row (two wells) per straight
            # pass, but the operator needs the accumulated result from both
            # rows.  The table therefore shows all four stable YOLO tracks;
            # only the boxes over the live camera remain frame-local.
            self.table = QTableWidget(4, 5)
            self.table.setHorizontalHeaderLabels(
                ["轨迹", "识别值", "置信度", "稳定帧", "估算 N/E (m)"]
            )
            self.table.horizontalHeader().setSectionResizeMode(
                QHeaderView.Stretch
            )
            self.table.verticalHeader().setVisible(False)
            self.table.setMaximumHeight(190)
            outer.addWidget(self.banner)
            outer.addWidget(self.image, 1)
            outer.addWidget(self.detail)
            outer.addWidget(self.table)
            self.setCentralWidget(root)
            self.setStyleSheet(
                """
                QMainWindow,QWidget{background:#ffffff;color:#20252b;font-size:14px}
                #banner{background:#eef2f6;color:#26313d;padding:12px;font-size:16px;font-weight:800;border:1px solid #d5dbe2}
                #camera{background:#f4f6f8;color:#5d6670;border:1px solid #c8ced6}
                QTableWidget{background:#ffffff;color:#20252b;gridline-color:#d8dde3;border:1px solid #c8ced6}
                QHeaderView::section{background:#eef2f6;color:#20252b;padding:8px;border:0;border-right:1px solid #d8dde3}
                """
            )
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.setTimerType(Qt.PreciseTimer)
            self.timer.start(33)

        def _refresh_frame(self) -> None:
            detail_path = RUNTIME_DIR / 'camera_detail.jpg'
            try:
                detail_time = detail_path.stat().st_mtime_ns
            except FileNotFoundError:
                self.detail.setText('靶标局部预览：当前画面没有已定位目标')
                self.detail_mtime_ns = -1
            else:
                if detail_time != self.detail_mtime_ns:
                    detail = QPixmap(str(detail_path))
                    if not detail.isNull():
                        self.detail.setPixmap(detail)
                        self.detail_mtime_ns = detail_time
            try:
                mtime_ns = self.frame_path.stat().st_mtime_ns
            except FileNotFoundError:
                return
            if mtime_ns != self.frame_mtime_ns:
                candidate = QPixmap(str(self.frame_path))
                if not candidate.isNull():
                    self.source_pixmap = candidate
                    self.frame_mtime_ns = mtime_ns
                    self.last_scaled_size = None
            size = self.image.size()
            if not self.source_pixmap.isNull() and size != self.last_scaled_size:
                self.image.setPixmap(
                    self.source_pixmap.scaled(
                        size,
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
                self.last_scaled_size = size

        def refresh(self) -> None:
            vision, changed = self.vision_cache.load()
            if changed:
                banner = vision.get("banner", "WAITING")
                self.banner.setText(banner)
                danger = any(
                    word in banner
                    for word in ("FAILED", "MISSING", "DISABLED")
                )
                self.banner.setStyleSheet(
                    "background:#fde8e8;color:#8a1c1c;padding:12px;font-weight:800;border:1px solid #e7b4b4"
                    if danger
                    else "background:#e8f6ec;color:#206b36;padding:12px;font-weight:800;border:1px solid #b8ddc3"
                )
                rows = vision.get("detections", [])
                for row in range(4):
                    values = rows[row] if row < len(rows) else {}
                    coordinates = (
                        f"{float(values['north_m']):.1f} / "
                        f"{float(values['east_m']):.1f}"
                        if values.get("north_m") is not None
                        and values.get("east_m") is not None
                        else "—"
                    )
                    cells = [
                        values.get("track_id", "—"),
                        str(values.get("label") or values.get("value", "—")),
                        (
                            f"{float(values['confidence']):.3f}"
                            if values.get("confidence") is not None
                            else "—"
                        ),
                        str(values.get("frames", "—")),
                        coordinates,
                    ]
                    for column, value in enumerate(cells):
                        self.table.setItem(
                            row,
                            column,
                            QTableWidgetItem(value),
                        )
            self._refresh_frame()

    app = QApplication.instance() or QApplication(sys.argv)
    window = Window()
    window.show()
    smoke_ms = int(os.environ.get("CUADC_GUI_SMOKE_MS", "0"))
    screenshot = os.environ.get("CUADC_GUI_SCREENSHOT")
    if screenshot:
        screenshot_path = Path(screenshot).expanduser().resolve()

        def save_smoke_screenshot() -> None:
            screenshot_path.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(screenshot_path), "PNG")

        QTimer.singleShot(500, save_smoke_screenshot)
    if smoke_ms > 0:
        QTimer.singleShot(smoke_ms, app.quit)
    return app.exec()
