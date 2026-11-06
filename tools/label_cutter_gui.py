#!/usr/bin/env python3
"""Label Cutter Control - a single-file PyQt5 host for the dual-head label cutter
Grbl_ESP32 firmware (see doc/LabelCutter.md).

    pip install pyqt5 pyserial
    python label_cutter_gui.py

Works on either UART (USB console or master link), 115200 8N1.
Lines are streamed with Grbl send-response flow control (one line in flight,
wait for ok/error). Realtime bytes (?, !, ~, Ctrl-X, jog cancel) bypass the queue.
"""
import html
import re
import sys
from collections import deque

import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt

BAUD = 115200
POLL_MS = 200  # status poll rate (5 Hz)

RT_STATUS = b"?"
RT_HOLD = b"!"
RT_RESUME = b"~"
RT_RESET = b"\x18"
RT_JOG_CANCEL = b"\x85"

ERRORS = {
    1: "Expected command letter", 2: "Bad number format", 3: "Invalid $ statement",
    4: "Negative value", 5: "Homing not enabled", 6: "Step pulse too short",
    7: "Settings read failed", 8: "Not idle", 9: "Locked by alarm ($X / $H)",
    10: "Soft limits need homing", 11: "Line too long / RX overrun",
    12: "Step rate too high", 14: "Line too long", 15: "Jog beyond soft limit",
    16: "Invalid jog command", 20: "Unsupported command", 21: "Modal group violation",
    22: "Feed rate not set", 23: "Integer value required", 24: "Conflicting axis commands",
    25: "Repeated word", 26: "No axis words", 27: "Invalid line number",
    28: "Missing value word", 29: "Unsupported WCS", 30: "G53 needs G0/G1",
    31: "Unused axis words", 32: "Arc needs in-plane axis", 33: "Invalid motion target",
    34: "Arc radius error", 35: "Arc needs offset", 36: "Unused value words",
    37: "Tool offset axis error", 38: "Tool number too high",
    150: "No paper", 151: "Sensor timeout (jam / misfeed)",
    152: "Paper ran out before rear sensor", 153: "Front sensor not covered",
    154: "Sequence aborted",
}
ALARMS = {
    1: "Hard limit (X switch)", 2: "Soft limit (X outside 0..$130)",
    3: "Reset during motion", 6: "Homing reset", 8: "Homing pull-off failed",
    9: "Homing switch not found",
}
SETTINGS = {
    0: "Step pulse, us", 1: "Step idle delay, ms", 2: "Step invert mask",
    3: "Direction invert mask", 4: "Enable invert mask (X,Y,Z)", 5: "Invert X limit",
    10: "Status report mask", 11: "Junction deviation, mm", 12: "Arc tolerance, mm",
    13: "Report inches", 20: "Soft limits (X)", 21: "Hard limit (X)",
    22: "Homing enable", 23: "Homing dir mask (bit0: switch at X=0)",
    24: "Homing feed, mm/min", 25: "Homing seek, mm/min", 26: "Homing debounce, ms",
    27: "Homing pull-off, mm", 30: "S for 100% head power",
    100: "X steps/mm", 101: "Y steps/mm", 102: "Z (roller) steps/mm",
    110: "X max rate, mm/min", 111: "Y max rate, mm/min", 112: "Z max rate, mm/min",
    120: "X accel, mm/s2", 121: "Y accel, mm/s2", 122: "Z accel, mm/s2",
    130: "X max travel, mm", 300: "DR: travel after rear sensor, mm",
    301: "DF: travel after front sensor, mm", 302: "Sequence paper speed, mm/min",
    303: "Max sensor search, mm", 304: "Extra eject travel, mm",
    305: "Sensor invert mask", 306: "Zero Y/Z after feed/eject",
    307: "Head pull-in time, ms", 308: "Head switch delay, ms",
    309: "Dir-to-step delay, us",
}
SETTING_RE = re.compile(r"^\$(\d+)=(.*)$")

STATE_COLORS = {
    "Idle": "#2e7d32", "Run": "#1565c0", "Jog": "#1565c0", "Hold": "#ef6c00",
    "Alarm": "#c62828", "Home": "#6a1b9a", "Check": "#616161",
}
C_SENT, C_OK, C_ERR, C_MSG, C_STATUS, C_OTHER = "#9cdcfe", "#6a9955", "#f44747", "#dcdcaa", "#808080", "#d4d4d4"


class SerialReader(QtCore.QThread):
    """Reads the port in the background and emits one signal per received line."""
    line = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._run = True

    def run(self):
        buf = b""
        while self._run:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:  # port unplugged etc.
                if self._run:
                    self.failed.emit(str(e))
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").strip()
                if text:
                    self.line.emit(text)

    def stop(self):
        self._run = False
        self.wait(1000)


class HistoryLineEdit(QtWidgets.QLineEdit):
    """Line edit with Up/Down command history."""

    def __init__(self):
        super().__init__()
        self.hist, self.idx = [], 0

    def push(self, text):
        if text and (not self.hist or self.hist[-1] != text):
            self.hist.append(text)
        self.idx = len(self.hist)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Up and self.hist:
            self.idx = max(0, self.idx - 1)
            self.setText(self.hist[self.idx])
        elif e.key() == Qt.Key_Down and self.hist:
            self.idx = min(len(self.hist), self.idx + 1)
            self.setText(self.hist[self.idx] if self.idx < len(self.hist) else "")
        else:
            super().keyPressEvent(e)


def indicator(text):
    lbl = QtWidgets.QLabel(text)
    lbl.setAlignment(Qt.AlignCenter)
    lbl.setMinimumWidth(56)
    set_indicator(lbl, False)
    return lbl


def set_indicator(lbl, on, color="#2e7d32"):
    bg = color if on else "#5a5a5a"
    lbl.setStyleSheet(f"QLabel{{background:{bg};color:white;border-radius:4px;padding:3px 6px;font-weight:bold}}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Label Cutter Control")
        self.resize(1200, 760)

        self.ser = None
        self.reader = None
        self.queue = deque()   # (line, source) waiting to be sent; source: "user" | "ui" | "job"
        self.pending = None    # (line, source) sent, waiting for ok/error
        self.state = "?"
        self.wco = [0.0, 0.0, 0.0]
        self.job_lines = []
        self.job_done = 0
        self.job_running = False
        self.setting_rows = {}
        self._updating_table = False

        self.poll = QtCore.QTimer(self)
        self.poll.timeout.connect(lambda: self.rt(RT_STATUS))

        self.build_ui()
        self.refresh_ports()
        self.set_connected(False)

    # ------------------------------------------------------------------ UI
    def build_ui(self):
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)

        # Connection bar
        bar = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(280)
        btn_refresh = QtWidgets.QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        bar.addWidget(QtWidgets.QLabel("Port:"))
        bar.addWidget(self.port_combo)
        bar.addWidget(btn_refresh)
        bar.addWidget(self.btn_connect)
        bar.addStretch()
        root.addLayout(bar)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        # Left column: status + controls
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self.build_status())
        self.controls = QtWidgets.QWidget()
        cv = QtWidgets.QVBoxLayout(self.controls)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.addWidget(self.build_machine())
        cv.addWidget(self.build_jog())
        cv.addWidget(self.build_tools())
        lv.addWidget(self.controls)
        lv.addStretch()
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setMinimumWidth(470)
        body.addWidget(scroll)

        # Right: tabs
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.build_console(), "Console")
        self.tabs.addTab(self.build_settings(), "Settings")
        self.tabs.addTab(self.build_job(), "Job")
        body.addWidget(self.tabs, 1)

        self.setCentralWidget(central)
        self.statusBar().showMessage("Disconnected")

    def build_status(self):
        g = QtWidgets.QGroupBox("Status")
        v = QtWidgets.QVBoxLayout(g)

        self.lbl_state = QtWidgets.QLabel("—")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setStyleSheet("font-size:20px;font-weight:bold;color:white;background:#5a5a5a;border-radius:6px;padding:6px")
        v.addWidget(self.lbl_state)

        grid = QtWidgets.QGridLayout()
        for c, name in enumerate(["X (knife)", "Y (sheet)", "Z (roller)"], start=1):
            h = QtWidgets.QLabel(name)
            h.setAlignment(Qt.AlignCenter)
            grid.addWidget(h, 0, c)
        self.lbl_mpos, self.lbl_wpos = [], []
        for r, (name, store) in enumerate([("Machine", self.lbl_mpos), ("Work", self.lbl_wpos)], start=1):
            grid.addWidget(QtWidgets.QLabel(name), r, 0)
            for c in range(3):
                lbl = QtWidgets.QLabel("0.000")
                lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                lbl.setStyleSheet("font-family:Consolas,monospace;font-size:16px;font-weight:bold")
                grid.addWidget(lbl, r, c + 1)
                store.append(lbl)
        v.addLayout(grid)

        row = QtWidgets.QHBoxLayout()
        self.ind_paper = indicator("Paper")
        self.ind_rear = indicator("Rear")
        self.ind_front = indicator("Front")
        self.ind_limit = indicator("X limit")
        self.ind_h1 = indicator("Head 1")
        self.ind_h2 = indicator("Head 2")
        for w in (self.ind_paper, self.ind_rear, self.ind_front, self.ind_limit, self.ind_h1, self.ind_h2):
            row.addWidget(w)
        v.addLayout(row)

        self.lbl_info = QtWidgets.QLabel("")
        self.lbl_info.setStyleSheet("font-family:Consolas,monospace")
        self.lbl_info.setWordWrap(True)
        v.addWidget(self.lbl_info)
        return g

    def build_machine(self):
        g = QtWidgets.QGroupBox("Machine")
        grid = QtWidgets.QGridLayout(g)
        buttons = [
            ("Home ($H)", lambda: self.send("$H", "ui")),
            ("Unlock ($X)", lambda: self.send("$X", "ui")),
            ("Feed sheet", lambda: self.send("$FEED", "ui")),
            ("Eject sheet", lambda: self.send("$EJECT", "ui")),
            ("Hold (!)", lambda: self.rt(RT_HOLD)),
            ("Resume (~)", lambda: self.rt(RT_RESUME)),
        ]
        for i, (text, fn) in enumerate(buttons):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(fn)
            grid.addWidget(b, i // 3, i % 3)
        b = QtWidgets.QPushButton("RESET (Ctrl-X)")
        b.setStyleSheet("QPushButton{background:#c62828;color:white;font-weight:bold;padding:6px}")
        b.clicked.connect(self.soft_reset)
        grid.addWidget(b, 2, 0, 1, 3)
        return g

    def build_jog(self):
        g = QtWidgets.QGroupBox("Jog")
        v = QtWidgets.QVBoxLayout(g)

        opts = QtWidgets.QHBoxLayout()
        self.jog_step = QtWidgets.QComboBox()
        self.jog_step.addItems(["0.1", "1", "5", "10", "50", "100"])
        self.jog_step.setCurrentText("10")
        self.jog_feed = QtWidgets.QSpinBox()
        self.jog_feed.setRange(10, 20000)
        self.jog_feed.setSingleStep(100)
        self.jog_feed.setValue(3000)
        opts.addWidget(QtWidgets.QLabel("Step (mm):"))
        opts.addWidget(self.jog_step)
        opts.addWidget(QtWidgets.QLabel("Feed (mm/min):"))
        opts.addWidget(self.jog_feed)
        opts.addStretch()
        v.addLayout(opts)

        grid = QtWidgets.QGridLayout()
        def jb(text, axis, sign, r, c):
            b = QtWidgets.QPushButton(text)
            b.setMinimumHeight(36)
            b.clicked.connect(lambda: self.jog(axis, sign))
            grid.addWidget(b, r, c)
        jb("Y+ ▲", "Y", 1, 0, 1)
        jb("◀ X−", "X", -1, 1, 0)
        jb("X+ ▶", "X", 1, 1, 2)
        jb("Y− ▼", "Y", -1, 2, 1)
        jb("Roller in (Z+)", "Z", 1, 0, 3)
        jb("Roller out (Z−)", "Z", -1, 2, 3)
        stop = QtWidgets.QPushButton("Stop jog")
        stop.clicked.connect(lambda: self.rt(RT_JOG_CANCEL))
        grid.addWidget(stop, 1, 1)
        v.addLayout(grid)

        zero = QtWidgets.QHBoxLayout()
        zero.addWidget(QtWidgets.QLabel("Zero work:"))
        for axes in ("X", "Y", "Z", "XYZ"):
            b = QtWidgets.QPushButton(axes if axes != "XYZ" else "All")
            words = " ".join(f"{a}0" for a in axes)
            b.clicked.connect(lambda _=False, w=words: self.send(f"G10 L20 P1 {w}", "ui"))
            zero.addWidget(b)
        b = QtWidgets.QPushButton("Go to work 0 (XY)")
        b.clicked.connect(lambda: self.send("G90 G0 X0 Y0", "ui"))
        zero.addWidget(b)
        v.addLayout(zero)
        return g

    def build_tools(self):
        g = QtWidgets.QGroupBox("Heads && fans")
        grid = QtWidgets.QGridLayout(g)

        self.head_s = QtWidgets.QSpinBox()
        self.head_s.setRange(0, 1000)
        self.head_s.setValue(1000)
        grid.addWidget(QtWidgets.QLabel("Head power S:"), 0, 0)
        grid.addWidget(self.head_s, 0, 1)
        for c, (text, code) in enumerate([("Head 1 down (M3)", "M3"), ("Head 2 down (M4)", "M4")]):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _=False, m=code: self.send(f"{m} S{self.head_s.value()}", "ui"))
            grid.addWidget(b, 1, c)
        b = QtWidgets.QPushButton("Heads up (M5)")
        b.clicked.connect(lambda: self.send("M5", "ui"))
        grid.addWidget(b, 1, 2)

        for i in range(2):
            lbl = QtWidgets.QLabel(f"Fan {i + 1}: 0 %")
            s = QtWidgets.QSlider(Qt.Horizontal)
            s.setRange(0, 100)
            s.valueChanged.connect(lambda val, l=lbl, n=i: l.setText(f"Fan {n + 1}: {val} %"))
            s.sliderReleased.connect(lambda sl=s, n=i: self.send(f"M68 E{n} Q{sl.value()}", "ui"))
            grid.addWidget(lbl, 2 + i, 0)
            grid.addWidget(s, 2 + i, 1, 1, 2)
        return g

    def build_console(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(5000)
        self.console.setStyleSheet("QPlainTextEdit{background:#1e1e1e;color:#d4d4d4;font-family:Consolas,monospace;font-size:12px}")
        v.addWidget(self.console, 1)

        row = QtWidgets.QHBoxLayout()
        self.cmd = HistoryLineEdit()
        self.cmd.setPlaceholderText("G-code or $ command  (Up/Down = history)")
        self.cmd.returnPressed.connect(self.send_console)
        self.btn_send = QtWidgets.QPushButton("Send")
        self.btn_send.clicked.connect(self.send_console)
        row.addWidget(self.cmd, 1)
        row.addWidget(self.btn_send)
        v.addLayout(row)

        quick = QtWidgets.QHBoxLayout()
        self.console_quick = []
        for c in ("$$", "$#", "$G", "$I", "$"):
            b = QtWidgets.QPushButton(c)
            b.clicked.connect(lambda _=False, cmd=c: self.send(cmd, "user"))
            quick.addWidget(b)
            self.console_quick.append(b)
        quick.addStretch()
        self.chk_status = QtWidgets.QCheckBox("Show status reports")
        quick.addWidget(self.chk_status)
        clr = QtWidgets.QPushButton("Clear")
        clr.clicked.connect(self.console.clear)
        quick.addWidget(clr)
        v.addLayout(quick)
        return w

    def build_settings(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        row = QtWidgets.QHBoxLayout()
        self.btn_read_settings = QtWidgets.QPushButton("Read settings ($$)")
        self.btn_read_settings.clicked.connect(lambda: self.send("$$", "ui"))
        row.addWidget(self.btn_read_settings)
        row.addWidget(QtWidgets.QLabel("Double-click a value to edit; it is written with $n=v on Enter."))
        row.addStretch()
        v.addLayout(row)
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Setting", "Value", "Description"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.table.itemChanged.connect(self.setting_edited)
        v.addWidget(self.table, 1)
        return w

    def build_job(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        row = QtWidgets.QHBoxLayout()
        btn_open = QtWidgets.QPushButton("Open G-code…")
        btn_open.clicked.connect(self.open_job)
        self.lbl_job = QtWidgets.QLabel("No file loaded")
        row.addWidget(btn_open)
        row.addWidget(self.lbl_job, 1)
        v.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.btn_job_start = QtWidgets.QPushButton("Start")
        self.btn_job_start.clicked.connect(self.start_job)
        self.btn_job_pause = QtWidgets.QPushButton("Pause (!)")
        self.btn_job_pause.clicked.connect(lambda: self.rt(RT_HOLD))
        self.btn_job_resume = QtWidgets.QPushButton("Resume (~)")
        self.btn_job_resume.clicked.connect(lambda: self.rt(RT_RESUME))
        self.btn_job_stop = QtWidgets.QPushButton("Stop (reset)")
        self.btn_job_stop.clicked.connect(self.soft_reset)
        for b in (self.btn_job_start, self.btn_job_pause, self.btn_job_resume, self.btn_job_stop):
            row.addWidget(b)
        self.chk_feed_first = QtWidgets.QCheckBox("$FEED before")
        self.chk_eject_after = QtWidgets.QCheckBox("$EJECT after")
        row.addWidget(self.chk_feed_first)
        row.addWidget(self.chk_eject_after)
        row.addStretch()
        v.addLayout(row)

        self.progress = QtWidgets.QProgressBar()
        v.addWidget(self.progress)
        self.job_view = QtWidgets.QPlainTextEdit()
        self.job_view.setReadOnly(True)
        self.job_view.setStyleSheet("QPlainTextEdit{font-family:Consolas,monospace}")
        v.addWidget(self.job_view, 1)
        return w

    # ------------------------------------------------------------ connection
    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        i = self.port_combo.findData(current)
        if i >= 0:
            self.port_combo.setCurrentIndex(i)

    def toggle_connect(self):
        if self.ser:
            self.disconnect_port()
            return
        port = self.port_combo.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, "Connect", "No serial port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.05, write_timeout=1)
        except Exception as e:
            self.ser = None
            QtWidgets.QMessageBox.critical(self, "Connect", f"Cannot open {port}:\n{e}")
            return
        self.reader = SerialReader(self.ser)
        self.reader.line.connect(self.on_line)
        self.reader.failed.connect(self.on_serial_failed)
        self.reader.start()
        self.poll.start(POLL_MS)
        self.set_connected(True)
        self.log(f"Connected to {port} @ {BAUD}", C_MSG)

    def disconnect_port(self):
        self.poll.stop()
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.abort_queue("Disconnected")
        self.set_connected(False)
        self.log("Disconnected", C_MSG)

    def on_serial_failed(self, msg):
        self.disconnect_port()
        QtWidgets.QMessageBox.critical(self, "Serial error", msg)

    def set_connected(self, on):
        self.btn_connect.setText("Disconnect" if on else "Connect")
        self.port_combo.setEnabled(not on)
        for w in [self.controls, self.cmd, self.btn_send, self.btn_read_settings, self.table,
                  self.btn_job_pause, self.btn_job_resume, self.btn_job_stop, *self.console_quick]:
            w.setEnabled(on)
        self.update_job_buttons()
        if not on:
            self.lbl_state.setText("Disconnected")
            self.lbl_state.setStyleSheet("font-size:20px;font-weight:bold;color:white;background:#5a5a5a;border-radius:6px;padding:6px")
            self.statusBar().showMessage("Disconnected")

    def closeEvent(self, e):
        if self.ser:
            self.disconnect_port()
        e.accept()

    # ------------------------------------------------------------ sending
    def write(self, data):
        if not self.ser:
            return False
        try:
            self.ser.write(data)
            return True
        except Exception as e:
            self.on_serial_failed(str(e))
            return False

    def rt(self, byte):
        """Realtime command: written immediately, never queued."""
        self.write(byte)

    def send(self, line, source="user"):
        line = line.strip()
        if not line or not self.ser:
            return
        self.queue.append((line, source))
        self.pump()

    def pump(self):
        if self.pending is not None or not self.queue or not self.ser:
            return
        line, source = self.queue.popleft()
        self.pending = (line, source)
        if source != "job":
            self.log("> " + line, C_SENT)
        self.write((line + "\n").encode())

    def send_console(self):
        text = self.cmd.text().strip()
        if not text:
            return
        self.cmd.push(text)
        self.cmd.clear()
        if text in ("?", "!", "~"):
            self.log("> " + text + " (realtime)", C_SENT)
            self.rt(text.encode())
        else:
            self.send(text, "user")

    def soft_reset(self):
        self.log("> Ctrl-X (soft reset)", C_ERR)
        self.abort_queue("Reset")
        self.rt(RT_RESET)

    def jog(self, axis, sign):
        step = float(self.jog_step.currentText())
        self.send(f"$J=G91 G21 {axis}{sign * step:.3f} F{self.jog_feed.value()}", "ui")

    def abort_queue(self, reason):
        self.queue.clear()
        self.pending = None
        if self.job_running:
            self.stop_job(f"Job aborted: {reason}")

    # ------------------------------------------------------------ receiving
    def on_line(self, s):
        if s.startswith("<"):
            self.parse_status(s)
            if self.chk_status.isChecked():
                self.log(s, C_STATUS)
            return
        if s == "ok":
            self.finish_pending(True, s)
        elif s.startswith("error:"):
            code = self._code(s)
            self.log(f"{s}  — {ERRORS.get(code, 'Unknown error')}"
                     + (f"   [{self.pending[0]}]" if self.pending else ""), C_ERR)
            self.finish_pending(False, s)
        elif s.startswith("ALARM:"):
            code = self._code(s)
            text = f"{s}  — {ALARMS.get(code, 'Unknown alarm')}"
            self.log(text, C_ERR, bold=True)
            self.statusBar().showMessage(text)
            self.abort_queue(text)
        elif s.startswith("Grbl "):
            self.log(s, C_MSG, bold=True)
            self.abort_queue("Controller reset")
        elif s.startswith("[MSG:"):
            self.log(s, C_MSG)
            self.statusBar().showMessage(s[5:-1] if s.endswith("]") else s, 8000)
        else:
            m = SETTING_RE.match(s)
            if m:
                self.update_setting(int(m.group(1)), m.group(2).strip())
            self.log(s, C_OTHER)

    @staticmethod
    def _code(s):
        try:
            return int(s.split(":", 1)[1])
        except ValueError:
            return -1

    def finish_pending(self, ok, reply):
        item, self.pending = self.pending, None
        if item:
            line, source = item
            if source == "job":
                if ok:
                    self.job_done += 1
                    self.progress.setValue(self.job_done)
                    if self.job_done >= len(self.job_lines):
                        self.stop_job("Job finished")
                else:
                    self.stop_job(f"Job stopped at line {self.job_done + 1}: {line} → {reply}")
            elif ok:
                self.log("ok", C_OK)
        self.pump()

    def parse_status(self, s):
        body = s.strip("<>")
        parts = body.split("|")
        state = parts[0]
        fields = {}
        for p in parts[1:]:
            k, _, v = p.partition(":")
            fields[k] = v

        def vec(text):
            try:
                return [float(x) for x in text.split(",")[:3]]
            except ValueError:
                return None

        if "WCO" in fields and vec(fields["WCO"]):
            self.wco = vec(fields["WCO"])
        mpos = vec(fields["MPos"]) if "MPos" in fields else None
        wpos = vec(fields["WPos"]) if "WPos" in fields else None
        if mpos and not wpos:
            wpos = [m - o for m, o in zip(mpos, self.wco)]
        elif wpos and not mpos:
            mpos = [w + o for w, o in zip(wpos, self.wco)]
        if mpos:
            for i in range(3):
                self.lbl_mpos[i].setText(f"{mpos[i]:.3f}")
                self.lbl_wpos[i].setText(f"{wpos[i]:.3f}")

        self.state = state.split(":")[0]
        color = STATE_COLORS.get(self.state, "#5a5a5a")
        self.lbl_state.setText(state)
        self.lbl_state.setStyleSheet(f"font-size:20px;font-weight:bold;color:white;background:{color};border-radius:6px;padding:6px")

        sn = fields.get("Sn", "")
        set_indicator(self.ind_paper, "P" in sn)
        set_indicator(self.ind_rear, "R" in sn, "#1565c0")
        set_indicator(self.ind_front, "F" in sn, "#1565c0")
        set_indicator(self.ind_limit, "X" in fields.get("Pn", ""), "#c62828")
        hd = fields.get("Hd", "")
        set_indicator(self.ind_h1, hd == "1", "#ef6c00")
        set_indicator(self.ind_h2, hd == "2", "#ef6c00")

        info = []
        if "FS" in fields:
            f, _, sp = fields["FS"].partition(",")
            info.append(f"F {f}  S {sp}")
        info.append(f"Fans {fields.get('Fn', '0,0')}")
        if "Seq" in fields:
            info.append(f"Seq {fields['Seq']}")
        if "Ov" in fields:
            info.append(f"Ov {fields['Ov']}")
        if "Bf" in fields:
            info.append(f"Bf {fields['Bf']}")
        if "Ln" in fields:
            info.append(f"Ln {fields['Ln']}")
        self.lbl_info.setText("   ".join(info))
        self.update_job_buttons()

    # ------------------------------------------------------------ console
    def log(self, text, color=C_OTHER, bold=False):
        t = html.escape(text)
        if bold:
            t = f"<b>{t}</b>"
        self.console.appendHtml(f'<span style="color:{color};white-space:pre">{t}</span>')

    # ------------------------------------------------------------ settings
    def update_setting(self, num, value):
        self._updating_table = True
        row = self.setting_rows.get(num)
        if row is None:
            row = self.table.rowCount()
            self.table.insertRow(row)
            key = QtWidgets.QTableWidgetItem(f"${num}")
            key.setFlags(key.flags() & ~Qt.ItemIsEditable)
            desc = QtWidgets.QTableWidgetItem(SETTINGS.get(num, ""))
            desc.setFlags(desc.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, 0, key)
            self.table.setItem(row, 2, desc)
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(value))
            self.setting_rows[num] = row
        else:
            self.table.item(row, 1).setText(value)
        self._updating_table = False

    def setting_edited(self, item):
        if self._updating_table or item.column() != 1:
            return
        num = int(self.table.item(item.row(), 0).text()[1:])
        value = item.text().strip()
        self.send(f"${num}={value}", "ui")
        self.send(f"${num}", "ui")  # read back the stored value

    # ------------------------------------------------------------ job
    def open_job(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open G-code", "", "G-code (*.nc *.gcode *.gc *.ngc *.tap *.txt);;All files (*)")
        if not path:
            return
        lines = []
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for raw in f:
                line = re.sub(r"\(.*?\)", "", raw.split(";", 1)[0]).strip()
                if line and line != "%":
                    lines.append(line[:255])
        self.job_lines = lines
        self.job_done = 0
        self.progress.setMaximum(max(1, len(lines)))
        self.progress.setValue(0)
        self.lbl_job.setText(f"{path}  ({len(lines)} lines)")
        self.job_view.setPlainText("\n".join(lines))
        self.update_job_buttons()

    def start_job(self):
        if not self.job_lines or self.job_running:
            return
        if self.state != "Idle":
            QtWidgets.QMessageBox.warning(self, "Job", f"Machine must be Idle to start (state: {self.state}).")
            return
        self.job_done = 0
        self.progress.setValue(0)
        self.job_running = True
        if self.chk_feed_first.isChecked():
            self.send("$FEED", "ui")
        for line in self.job_lines:
            self.queue.append((line, "job"))
        self.log(f"Job started ({len(self.job_lines)} lines)", C_MSG, bold=True)
        self.update_job_buttons()
        self.pump()

    def stop_job(self, message):
        finished = message == "Job finished"
        self.job_running = False
        self.queue = deque(i for i in self.queue if i[1] != "job")
        if finished and self.chk_eject_after.isChecked():
            self.send("$EJECT", "ui")
        self.log(message, C_OK if finished else C_ERR, bold=True)
        self.statusBar().showMessage(message)
        self.update_job_buttons()

    def update_job_buttons(self):
        self.btn_job_start.setEnabled(bool(self.ser) and bool(self.job_lines) and not self.job_running)


def main():
    QtWidgets.QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
