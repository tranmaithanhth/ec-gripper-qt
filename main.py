import sys, json, queue, time, threading
from pathlib import Path
from dataclasses import asdict
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QColor, QPen
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,
 QFormLayout,QDoubleSpinBox,QSpinBox,QPushButton,QLabel,QComboBox,QCheckBox,QPlainTextEdit,QGroupBox,QDialog,QTabWidget,QScrollArea,QSlider,QSizePolicy)
from ports import discover, diagnose
from control import Settings
from backend import Backend
from visual import JawView
from mechanics import Geometry, read_config
from collections import deque

CONFIG=Path(__file__).with_name('hardware_settings.json')
MECHANICS_CONFIG=Path(__file__).with_name('mechanical_settings.json')

class Trace(QWidget):
    def __init__(self):
        super().__init__(); self.points=deque(maxlen=500); self.setMinimumHeight(140)
    def paintEvent(self,event):
        p=QPainter(self); p.fillRect(self.rect(),QColor('#ffffff'))
        p.setPen(QColor('#334155')); p.drawText(15,22,'Actual current (A), −3 … +3; last 25 s nominal')
        p.setPen(QColor('#cbd5e1')); mid=(self.height()+30)/2
        p.drawLine(15,int(mid),self.width()-15,int(mid))
        p.setPen(QPen(QColor('#087f72'),2)); data=list(self.points)
        for i in range(1,len(data)):
            if data[i-1] is None or data[i] is None: continue
            y1=mid-data[i-1]/3*(self.height()-50)/2; y2=mid-data[i]/3*(self.height()-50)/2
            p.drawLine(int(15+(i-1)*(self.width()-30)/500),int(y1),int(15+i*(self.width()-30)/500),int(y2))

class Window(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('EC Gripper Qt • Simulation / Real motor'); self.resize(1200,720)
        self.events=queue.Queue(); self.backend=Backend(self.events); self.backend.start()
        self._ports_initialized=False; self._connecting=False; self._port_signature=None; self._diagnosing=False
        self.fb=None; self.ready=False; self.active=False; self.pending=False
        self.setStyleSheet('QGroupBox {font-weight:bold;} QPushButton {padding:7px;}')
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)
        layout.addWidget(QLabel('EC-A4310-P2-36 + Waveshare USB-CAN-A | servo position + current threshold'))
        mode_row=QHBoxLayout(); layout.addLayout(mode_row)
        self.mode=QComboBox(); self.mode.addItem('Real EC-A4310 — USB-CAN','real'); self.mode.addItem('Simulation — no USB required','simulation'); mode_row.addWidget(self.mode)
        self.allports=QCheckBox('Show ttyS / all ports'); self.allports.toggled.connect(lambda _:self.refresh_ports()); mode_row.addWidget(self.allports)
        self.jaws=JawView(); self.jaws.setMinimumHeight(180); self.jaws.object_present=False; layout.addWidget(self.jaws)
        row=QHBoxLayout(); layout.addLayout(row)
        self.port=QComboBox(); self.port.setEditable(True); row.addWidget(self.port)
        self.refresh=QPushButton('Refresh ports'); self.refresh.clicked.connect(self.refresh_ports); row.addWidget(self.refresh)
        self.mid=QSpinBox(); self.mid.setRange(0,2046); self.mid.setValue(0); self.mid.setSpecialValueText('Auto ID'); row.addWidget(self.mid)
        self.connect_btn=QPushButton('Connect'); self.connect_btn.clicked.connect(self.connect_motor); row.addWidget(self.connect_btn)
        self.disconnect_btn=QPushButton('Disconnect'); self.disconnect_btn.clicked.connect(lambda:self.backend.request('disconnect')); row.addWidget(self.disconnect_btn)
        self.diagnostic_btn=QPushButton('Diagnose USB'); self.diagnostic_btn.clicked.connect(self.diagnose_usb); row.addWidget(self.diagnostic_btn)
        self.port_hint=QLabel(); self.port_hint.setWordWrap(True); layout.addWidget(self.port_hint)
        self.refresh_ports()
        groups=QHBoxLayout(); layout.addLayout(groups)
        calibration=QGroupBox('Calibration — measured motor angles'); groups.addWidget(calibration)
        c=QFormLayout(calibration)
        self.open=self.spin(c,'Open angle (deg)',-100000,100000,0,3)
        self.closed=self.spin(c,'Closed angle (deg)',-100000,100000,0,3)
        self.open_set=QPushButton('Use current angle as OPEN'); self.open_set.clicked.connect(lambda:self.capture(self.open)); c.addRow(self.open_set)
        self.close_set=QPushButton('Use current angle as CLOSED'); self.close_set.clicked.connect(lambda:self.capture(self.closed)); c.addRow(self.close_set)
        params=QGroupBox('Current thresholds — not force in N'); groups.addWidget(params); f=QFormLayout(params)
        self.speed=self.spin(f,'Speed (rpm)',.1,5,1,1)
        self.close_a=self.spin(f,'I_close / I_open max (A)',.1,3,.5,1)
        self.hold_a=self.spin(f,'I_hold threshold (A)',.1,3,.3,1)
        self.contact_a=self.spin(f,'Contact detection (A)',.1,3,.4,1)
        self.preload=self.spin(f,'Preload (deg)',0,5,1,2)
        self.timeout=self.spin(f,'Move timeout (s)',2,120,20,1)
        row=QHBoxLayout(); layout.addLayout(row)
        self.save=QPushButton('Save settings'); self.save.clicked.connect(self.save_settings); row.addWidget(self.save)
        self.motion_buttons=[]; row=QHBoxLayout(); layout.addLayout(row)
        self.raw=QSpinBox(); self.raw.setRange(0,1000); self.raw.setValue(700); row.addWidget(self.raw)
        for label,raw in [('Move target',None),('OPEN',0),('CLOSE',1000)]:
            b=QPushButton(label); b.clicked.connect(lambda checked=False,value=raw:self.backend.request('move',raw=self.raw.value() if value is None else value)); row.addWidget(b); b.setEnabled(False); self.motion_buttons.append(b)
        self.stop_btn=QPushButton('STOP — cease command stream'); self.stop_btn.setStyleSheet('background:#bd3030;color:white;font-weight:bold;'); self.stop_btn.clicked.connect(self.stop); row.addWidget(self.stop_btn)
        layout.addWidget(QLabel('STOP relies on existing motor heartbeat timeout (~500 ms); support payload before stopping.'))
        self.state=QLabel('DISCONNECTED'); layout.addWidget(self.state)
        self.feedback=QLabel('Position — | Current — | Temperature —'); layout.addWidget(self.feedback)
        self.trace=Trace(); layout.addWidget(self.trace)
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(600); layout.addWidget(self.log)
        # Rehouse existing controls without changing their signals or motor commands.
        shell=QWidget(); outer=QVBoxLayout(shell); outer.setContentsMargins(12,10,12,10); outer.setSpacing(6)
        title=QLabel('EC GRIPPER'); title.setObjectName('title'); header=QHBoxLayout(); header.addWidget(title); outer.addLayout(header)
        subtitle=QLabel('EC-A4310 · Điều khiển tay kẹp'); subtitle.setObjectName('muted'); header.addWidget(subtitle); header.addStretch()
        self.tabs=QTabWidget(); outer.addWidget(self.tabs)
        control_page=QWidget(); control_layout=QVBoxLayout(control_page)
        control_layout.setContentsMargins(10,8,10,8); control_layout.setSpacing(6)
        status_row=QHBoxLayout(); control_layout.addLayout(status_row)
        self.overview=QLabel('Chưa kết nối'); self.overview.setWordWrap(True); self.overview.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred); status_row.addWidget(self.overview,1)
        settings_link=QPushButton('Cài đặt kết nối →'); settings_link.clicked.connect(lambda:self.tabs.setCurrentIndex(1)); status_row.addWidget(settings_link)
        self.monitor_button=QPushButton('Theo dõi / Nhật ký ↗'); self.monitor_button.clicked.connect(self.show_monitor); status_row.addWidget(self.monitor_button)
        quick_row=QHBoxLayout(); control_layout.addLayout(quick_row)
        self.connect_real=QPushButton('Connect real gripper')
        self.connect_real.clicked.connect(lambda:self.quick_connect('real'))
        self.connect_simulator=QPushButton('Connect simulator')
        self.connect_simulator.clicked.connect(lambda:self.quick_connect('simulation'))
        self.quick_disconnect=QPushButton('Disconnect')
        self.quick_disconnect.clicked.connect(self.disconnect_btn.click)
        for button in (self.connect_real,self.connect_simulator,self.quick_disconnect):
            quick_row.addWidget(button)
        self.quick_disconnect.setEnabled(False)
        control_layout.addWidget(self.jaws,1)
        self.feedback.setWordWrap(True); control_layout.addWidget(self.feedback)
        self.aperture=QLabel('Độ mở: — mm · Chưa cấu hình cơ khí'); control_layout.addWidget(self.aperture)
        position_box=QWidget(); position_layout=QVBoxLayout(position_box); position_layout.setContentsMargins(0,0,0,0)
        position_row=QHBoxLayout(); position_layout.addLayout(position_row)
        position_row.addWidget(QLabel('Vị trí · 0 mở / 1000 đóng'))
        self.position_slider=QSlider(Qt.Horizontal); self.position_slider.setRange(0,1000); self.position_slider.setValue(self.raw.value())
        self.position_slider.setAccessibleName('Vị trí tay kẹp, mức đóng 0 đến 1000')
        self.position_slider.valueChanged.connect(self.raw.setValue); self.raw.valueChanged.connect(self.position_slider.setValue)
        position_row.addWidget(self.position_slider,1); position_row.addWidget(self.raw)
        self.motion_buttons[0].setText('ĐẾN VỊ TRÍ'); position_row.addWidget(self.motion_buttons[0])
        self.raw.setToolTip('Mức đóng theo hành trình góc; 500 không nhất thiết bằng nửa độ mở mm.')
        control_layout.addWidget(position_box)
        actions=QHBoxLayout(); control_layout.addLayout(actions)
        for button,label,name in [(self.motion_buttons[1],'MỞ · OPEN','openButton'),(self.motion_buttons[2],'KẸP · CLOSE','closeButton'),(self.stop_btn,'DỪNG · STOP','stopButton')]:
            button.setText(label); button.setObjectName(name); button.setMinimumHeight(44); actions.addWidget(button)
        self.stop_btn.setStyleSheet('')
        self.action_message=QLabel('Chọn Connect real gripper hoặc Connect simulator để bắt đầu.')
        self.action_message.setWordWrap(False); self.action_message.setFixedHeight(20); self.action_message.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Fixed); control_layout.addWidget(self.action_message)
        self.log.textChanged.connect(self.update_action_message)
        notice=QLabel('STOP ngừng gửi lệnh; động cơ dựa vào timeout khoảng 500 ms. Đỡ vật trước khi dừng. Kết nối lại hoặc Save settings để tiếp tục.')
        notice.setWordWrap(True); notice.setObjectName('muted'); control_layout.addWidget(notice)
        self.tabs.addTab(control_page,'Điều khiển')
        settings_page=QWidget(); settings_layout=QVBoxLayout(settings_page); settings_layout.setSpacing(14)
        connection=QGroupBox('Kết nối'); connection_layout=QFormLayout(connection)
        connection_layout.addRow('Chế độ',self.mode); connection_layout.addRow('Cổng USB-CAN',self.port)
        connection_layout.addRow('Motor ID',self.mid)
        connection_actions=QHBoxLayout()
        for widget in [self.refresh,self.connect_btn,self.disconnect_btn,self.diagnostic_btn]: connection_actions.addWidget(widget)
        connection_layout.addRow(connection_actions); connection_layout.addRow(self.allports); connection_layout.addRow(self.port_hint)
        settings_layout.addWidget(connection)
        settings_layout.addWidget(calibration); settings_layout.addWidget(params)
        settings_layout.addWidget(self.save)
        settings_hint=QLabel('Bấm DỪNG để sửa thông số. Save settings áp dụng thông số và bật lại điều khiển khi đã kết nối.')
        settings_hint.setWordWrap(True); settings_layout.addWidget(settings_hint)
        self.build_mechanics(settings_layout)
        position_layout.addWidget(self.target_aperture)
        self.monitor=QDialog(self,Qt.Window); self.monitor.setWindowTitle('EC Gripper · Theo dõi và nhật ký'); self.monitor.resize(850,650)
        monitor_layout=QVBoxLayout(self.monitor)
        self.state.setWordWrap(True); monitor_layout.addWidget(self.state)
        self.monitor_feedback=QLabel('Chờ phản hồi'); self.monitor_feedback.setWordWrap(True); monitor_layout.addWidget(self.monitor_feedback)
        self.monitor_aperture=QLabel('Độ mở: — mm'); self.monitor_aperture.setWordWrap(True); monitor_layout.addWidget(self.monitor_aperture)
        monitor_layout.addWidget(self.trace,1); monitor_layout.addWidget(self.log,2)
        monitor_stop=QPushButton('DỪNG · STOP'); monitor_stop.setObjectName('stopButton'); monitor_stop.clicked.connect(self.stop); monitor_layout.addWidget(monitor_stop)
        monitor_hint=QLabel('Đóng cửa sổ này chỉ ẩn phần theo dõi; tay kẹp vẫn hoạt động.'); monitor_hint.setWordWrap(True); monitor_layout.addWidget(monitor_hint)
        launch_monitor=QPushButton('Mở cửa sổ Theo dõi và nhật ký ↗'); launch_monitor.clicked.connect(self.show_monitor); settings_layout.addWidget(launch_monitor)
        settings_layout.addStretch()
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(settings_page)
        self.tabs.addTab(scroll,'Cài đặt')
        # STOP is always reachable, even while reviewing settings.
        self.settings_stop=QPushButton('DỪNG · STOP'); self.settings_stop.setObjectName('stopButton'); self.settings_stop.clicked.connect(self.stop)
        self.tabs.setCornerWidget(self.settings_stop)
        old_root=self.takeCentralWidget(); self.setCentralWidget(shell); old_root.deleteLater()
        self.setStyleSheet("""
            QWidget {background:#f4f7fb; color:#24364b; font-size:13px;}
            QLabel#title {font-size:21px; font-weight:700; color:#16324f;}
            QLabel#muted {color:#52677c; font-size:12px;}
            QTabWidget::pane {border:1px solid #ccd7e3; border-radius:8px;}
            QTabBar::tab {padding:8px 24px; background:#e8eef5; color:#52677c;}
            QTabBar::tab:selected {background:#ffffff; color:#087f72; border-bottom:3px solid #087f72;}
            QGroupBox {border:1px solid #ccd7e3; border-radius:8px; margin-top:16px; padding:16px; font-weight:bold;}
            QGroupBox::title {subcontrol-origin:margin; left:12px; padding:0 5px;}
            QPushButton {background:#ffffff; border:1px solid #b8c8d8; border-radius:7px; padding:6px 12px;}
            QPushButton:hover {background:#e3edf8; border-color:#7295b6;}
            QPushButton:pressed {background:#d1e2f2;}
            QPushButton#openButton {background:#216b96; color:white; font-size:17px; font-weight:bold;}
            QPushButton#closeButton {background:#087f72; color:white; font-size:17px; font-weight:bold;}
            QPushButton#stopButton {background:#b93849; color:white; font-weight:bold;}
            QPushButton#openButton:hover {background:#195678;}
            QPushButton#closeButton:hover {background:#06675d;}
            QPushButton#stopButton:hover {background:#9e2939;}
            QPushButton:disabled, QPushButton#openButton:disabled, QPushButton#closeButton:disabled {background:#e5eaf0; color:#68798b; border-color:#d0dae4;}
            QComboBox,QSpinBox,QDoubleSpinBox,QPlainTextEdit {background:#ffffff; color:#24364b; border:1px solid #b8c8d8; border-radius:4px; padding:6px; selection-background-color:#216b96; selection-color:white;}
            QComboBox QAbstractItemView {background:white; color:#24364b; selection-background-color:#d9eaf7; selection-color:#16324f;}
            QComboBox:disabled,QSpinBox:disabled,QDoubleSpinBox:disabled {background:#e9eef4; color:#68798b;}
            QSlider::groove:horizontal {height:6px; background:#cbd7e3; border-radius:3px;}
            QSlider::sub-page:horizontal {background:#216b96; border-radius:3px;}
            QSlider::handle:horizontal {background:#216b96; border:1px solid #185578; width:16px; margin:-5px 0; border-radius:8px;}
            QCheckBox::indicator {width:14px; height:14px; border:1px solid #8498ad; border-radius:3px; background:white;}
            QCheckBox::indicator:checked {background:#087f72; border:2px solid #055b52;}
            QCheckBox::indicator:disabled {background:#dce4ec; border-color:#a6b6c6;}
            QCheckBox::indicator:checked:disabled {background:#8bafaa;}
            QScrollArea {border:none;}
            QToolTip {background:#ffffff; color:#24364b; border:1px solid #b8c8d8;}
        """)
        self.config_controls=[self.open,self.closed,self.open_set,self.close_set,self.speed,self.close_a,self.hold_a,self.contact_a,self.preload,self.timeout,self.save]
        self.config_controls.extend(self.mechanical_controls)
        # Long feedback and geometry messages must wrap rather than widen the window.
        for label in (self.feedback,self.aperture,self.target_aperture):
            label.setWordWrap(True); label.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Preferred)
        self.load_mechanics()
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.mode_changed()
        self.timer=QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(50)
        self.port_timer=QTimer(self); self.port_timer.timeout.connect(self.auto_refresh_ports); self.port_timer.start(2000)
        available=self.screen().availableGeometry()
        self.resize(min(1200,available.width()-32),min(720,available.height()-48))
        self.move(available.x()+(available.width()-self.width())//2,available.y()+16)

    def update_action_message(self):
        message=self.log.document().lastBlock().text()
        self.action_message.setText(self.action_message.fontMetrics().elidedText(message,Qt.ElideRight,max(100,self.action_message.width())))
        self.action_message.setToolTip(message)

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'action_message'): self.update_action_message()

    def show_monitor(self):
        self.monitor.show(); self.monitor.raise_(); self.monitor.activateWindow()

    def build_mechanics(self,layout):
        self.geometry=None; self.mech_solutions=[]
        group=QGroupBox('Cơ khí · 5 thông số chính'); form=QFormLayout(group)
        hint=QLabel('Nhập độc lập các thông số dưới đây. Chương trình tự tính góc mở/đóng, không sửa các giá trị bạn nhập.')
        hint.setWordWrap(True); form.addRow(hint)
        self.mech_widgets={}
        fields=[('max_mm','Độ mở tối đa (mm)',0,10000,80),('r_mm','Tay quay r (mm)',0,10000,0),('l_mm','Thanh truyền l (mm)',0,10000,0),('e_mm','Lệch tâm e (mm)',-10000,10000,0),('doffset_mm','Doffset — số bù độ mở (mm)',-100000,100000,0)]
        for key,label,lo,hi,value in fields:
            self.mech_widgets[key]=self.spin(form,label,lo,hi,value,4)
            self.mech_widgets[key].setKeyboardTracking(False)
        note=QLabel('D = Doffset + 2x. Doffset là số bù có dấu, có thể âm; không phải độ lệch tâm e. CLOSED được tính tại D = 0 mm. Góc tự tính chỉ dùng hiển thị; góc encoder vẫn hiệu chuẩn riêng.')
        note.setWordWrap(True); form.addRow(note)
        self.mech_branch=QComboBox(); self.mech_branch.currentIndexChanged.connect(self.select_mechanical_branch)
        form.addRow(self.mech_branch)
        self.mech_status=QLabel(); self.mech_status.setWordWrap(True); form.addRow(self.mech_status)
        self.target_aperture=QLabel(); self.target_aperture.setWordWrap(True); form.addRow(self.target_aperture)
        self.mech_save=QPushButton('Lưu thông số cơ khí'); self.mech_save.clicked.connect(self.save_mechanics); form.addRow(self.mech_save)
        self.mechanical_controls=list(self.mech_widgets.values())+[self.mech_branch,self.mech_save]
        for widget in self.mech_widgets.values(): widget.valueChanged.connect(self.geometry_changed)
        self.raw.valueChanged.connect(self.update_target_aperture)
        layout.addWidget(group); self.geometry_changed()

    def geometry_changed(self,*_):
        preferred=[self.geometry.crank_open_deg,self.geometry.crank_closed_deg] if self.geometry else None
        self.geometry=None; self.mech_solutions=[]
        self.mech_branch.blockSignals(True); self.mech_branch.clear()
        try:
            inputs=Geometry(**{key:w.value() for key,w in self.mech_widgets.items()})
            self.mech_solutions=inputs.solve()
            for i,g in enumerate(self.mech_solutions):
                self.mech_branch.addItem(f'Nhánh {i+1}: OPEN {g.crank_open_deg:.3f}° → CLOSED {g.crank_closed_deg:.3f}°')
            self.mech_branch.setCurrentIndex(self.nearest_branch(preferred))
        except ValueError as exc: self.mech_status.setText('Chưa tính được: '+str(exc))
        self.mech_branch.blockSignals(False); self.mech_branch.setVisible(len(self.mech_solutions)>1)
        self.select_mechanical_branch()

    def nearest_branch(self,angles):
        if not angles or not self.mech_solutions: return 0
        def distance(g):
            # Preserve direction as well as the OPEN phase when migrating/reloading.
            return abs((g.crank_open_deg-angles[0]+180)%360-180)+abs((g.crank_closed_deg-g.crank_open_deg)-(angles[1]-angles[0]))
        return min(range(len(self.mech_solutions)),key=lambda i:distance(self.mech_solutions[i]))

    def select_mechanical_branch(self,*_):
        index=self.mech_branch.currentIndex()
        self.geometry=self.mech_solutions[index] if 0<=index<len(self.mech_solutions) else None
        if self.geometry:
            g=self.geometry
            text=f'Tự tính: góc OPEN {g.crank_open_deg:.3f}° · góc CLOSED {g.crank_closed_deg:.3f}° · hành trình mỗi ngón {g.max_mm/2:.2f} mm'
            if len(self.mech_solutions)>1: text+='\nCó nhiều nhánh hình học; chọn nhánh đúng với chiều lắp thực tế ở trên.'
            self.mech_status.setText(text)
        self.update_target_aperture()

    def update_target_aperture(self,*_):
        text=f'Mục tiêu {self.raw.value()}/1000 → {self.geometry.at_fraction(self.raw.value()/1000):.2f} mm (theo mô hình)' if self.geometry else f'Đã chọn {self.raw.value()}/1000 · Độ mở đích: — mm (chưa có mô hình cơ khí)'
        self.target_aperture.setText(text)

    def save_mechanics(self):
        try:
            data={key:w.value() for key,w in self.mech_widgets.items()}
            data['version']=2
            data['selected_angles']=[self.geometry.crank_open_deg,self.geometry.crank_closed_deg] if self.geometry else None
            temporary=MECHANICS_CONFIG.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8'); temporary.replace(MECHANICS_CONFIG)
            self.log.appendPlainText('Đã lưu thông số cơ khí.' if self.geometry else 'Đã lưu bản nháp cơ khí; chưa có nghiệm để hiển thị mm.')
        except (ValueError,OSError) as exc: self.log.appendPlainText('Cơ khí: '+str(exc))

    def load_mechanics(self):
        if not MECHANICS_CONFIG.exists(): return
        try:
            g,preferred=read_config(json.loads(MECHANICS_CONFIG.read_text(encoding='utf-8')))
            if any(not w.minimum()<=getattr(g,key)<=w.maximum() for key,w in self.mech_widgets.items()): raise ValueError('Thông số trong file nằm ngoài khoảng nhập hỗ trợ; file vẫn được giữ.')
            for key,w in self.mech_widgets.items():
                w.blockSignals(True); w.setValue(getattr(g,key)); w.blockSignals(False)
            self.geometry_changed()
            if preferred and self.mech_solutions: self.mech_branch.setCurrentIndex(self.nearest_branch(preferred))
        except (ValueError,TypeError,KeyError,OSError) as exc: self.log.appendPlainText('Không nạp được cơ khí: '+str(exc))

    def update_aperture(self,latest):
        self.jaws.width_mm=None; self.jaws.max_mm=None; self.jaws.object_width_mm=None
        text='Độ mở: — mm · Chưa đủ thông số cơ khí hợp lệ'
        if self.geometry:
            try:
                self.settings()
                if not self.fb: raise ValueError('Chưa có phản hồi encoder')
                width=self.geometry.width(self.fb['position'],self.open.value(),self.closed.value())
                self.jaws.width_mm=width; self.jaws.max_mm=self.geometry.max_mm
                self.jaws.object_width_mm=self.geometry.at_fraction(.55)
                text=f'Độ mở theo mô hình: {width:.2f} mm / {self.geometry.max_mm:.2f} mm'
                if self.jaws.stale: text+=' · Phản hồi cũ'
                if latest['target'] is not None:
                    try: text+=f" · Đích: {self.geometry.width(latest['target'],self.open.value(),self.closed.value()):.2f} mm"
                    except ValueError: text+=' · Đích ngoài khoảng hiệu chuẩn'
            except ValueError as exc: text='Độ mở: — mm · '+str(exc)
        self.aperture.setText(text); self.monitor_aperture.setText(text)
        self.monitor_feedback.setText(self.feedback.text())

    def spin(self,form,name,lo,hi,value,decimals):
        s=QDoubleSpinBox(); s.setDecimals(decimals); s.setRange(lo,hi); s.setValue(value); s.setSingleStep(10**-decimals); form.addRow(name,s); return s
    def refresh_ports(self, automatic=False):
        if automatic and (self.ready or self._connecting or self.port.hasFocus() or self.port.view().isVisible()): return
        old=self.port.currentText().split(' | ',1)[0].strip()
        ports,errors=discover(self.allports.isChecked())
        signature=tuple((p.device,p.label,tuple(p.aliases)) for p in ports)
        if automatic and signature==self._port_signature: return
        self._port_signature=signature
        self.port.blockSignals(True); self.port.clear()
        for p in ports:
            self.port.addItem(p.label,p.device)
            self.port.setItemData(self.port.count()-1,f'{p.hwid}\nAliases: {", ".join(p.aliases)}',Qt.ToolTipRole)
        if old:
            index=next((i for i,p in enumerate(ports) if p.device==old),-1)
            if index>=0: self.port.setCurrentIndex(index)
            elif self._ports_initialized: self.port.setCurrentText(old)
        self.port.blockSignals(False); self._ports_initialized=True
        count=sum(p.usb for p in ports)
        mode_note='Simulation uses no USB. Select Real EC-A4310 to connect hardware.' if self.mode.currentData()=='simulation' else 'Real motor: select the USB serial port, then Connect hardware.'
        self.port_hint.setText(f'{mode_note} Detected: {count} USB serial / {len(ports)} listed. '+('No USB endpoint: use Diagnose USB.' if not count else 'Listed ports may still be busy.'))
        if errors and hasattr(self,'log'): self.log.appendPlainText('; '.join(errors))
    def auto_refresh_ports(self): self.refresh_ports(automatic=True)
    def diagnose_usb(self):
        if self._diagnosing: return
        self._diagnosing=True; self.diagnostic_btn.setEnabled(False)
        selected=self.port.currentText().split(' | ',1)[0].strip()
        def work():
            try: report=diagnose(selected)
            except Exception as e: report='Diagnostic error: '+str(e)
            self.events.put(('diagnostic',{'report':report}))
        threading.Thread(target=work,daemon=True).start()
    def show_diagnostic(self,report):
        self._diagnosing=False; self.diagnostic_btn.setEnabled(True)
        dialog=QDialog(self); dialog.setWindowTitle('USB diagnostics — copy this report'); dialog.resize(850,600)
        box=QVBoxLayout(dialog); edit=QPlainTextEdit(); edit.setReadOnly(True); edit.setPlainText(report); box.addWidget(edit)
        copy=QPushButton('Copy report'); copy.clicked.connect(lambda:QApplication.clipboard().setText(report)); box.addWidget(copy)
        close=QPushButton('Close'); close.clicked.connect(dialog.accept); box.addWidget(close)
        dialog.setAttribute(Qt.WA_DeleteOnClose); dialog.show()
    def update_connection_buttons(self):
        available=not self.ready and not self._connecting
        for button in (self.connect_btn,self.connect_real,self.connect_simulator):
            button.setEnabled(available)
        self.quick_disconnect.setEnabled(self.ready and not self._connecting)
        self.connect_real.setText('Đang kết nối…' if self._connecting and self.mode.currentData()=='real' else 'Connect real gripper')
        self.connect_simulator.setText('Đang kết nối…' if self._connecting and self.mode.currentData()=='simulation' else 'Connect simulator')

    def quick_connect(self,mode):
        if self.ready or self._connecting: return
        self.mode.setCurrentIndex(self.mode.findData(mode))
        if mode=='real' and not self.port.currentText().strip():
            self.tabs.setCurrentIndex(1); self.port.setFocus()
        self.connect_motor()

    def connect_motor(self):
        if self.ready or self._connecting: return
        if self.mode.currentData()=='real' and not self.port.currentText().strip(): self.log.appendPlainText('Select USB-CAN serial port'); return
        self._connecting=True; self.mode.setEnabled(False); self.connect_btn.setEnabled(False); self.backend.request('connect',port=self.port.currentText().split(' | ',1)[0].strip(),motor_id=self.mid.value() or None,mode=self.mode.currentData())
        self.backend.request('sim_object',present=False)
        self.update_connection_buttons()
    def mode_changed(self):
        sim=self.mode.currentData()=='simulation'
        self.port.setEnabled(True); self.mid.setEnabled(not sim)
        self.connect_btn.setText('Connect simulation' if sim else 'Connect hardware')
        self.refresh_ports()
        self.jaws.mode=self.mode.currentData(); self.jaws.update()
        if sim:
            self.open.setValue(0); self.closed.setValue(90)
            self.speed.setValue(2); self.close_a.setValue(.5); self.hold_a.setValue(.3); self.contact_a.setValue(.4)
        else:
            self.open.setValue(0); self.closed.setValue(0)
            self.load_settings()
            self.log.appendPlainText('Động cơ thật: dùng góc OPEN/CLOSED đã lưu; kết nối thành công sẽ tự bật điều khiển và UDP.')
    def capture(self,widget):
        if self.fb and self.backend.can and time.monotonic()-self.backend.fb_time<.5:
            widget.setValue(self.fb['position'])
        else: self.log.appendPlainText('Connect and obtain fresh position first')
    def settings(self):
        s=Settings(open_deg=self.open.value(),closed_deg=self.closed.value(),speed_rpm=self.speed.value(),close_a=self.close_a.value(),hold_a=self.hold_a.value(),contact_a=self.contact_a.value(),preload_deg=self.preload.value(),move_timeout_s=self.timeout.value())
        s.validate(); return s
    def save_settings(self):
        try:
            s=self.settings()
            if self.mode.currentData()=='simulation':
                self.log.appendPlainText('Đã áp dụng thông số mô phỏng; không ghi đè cấu hình phần cứng.')
            else:
                CONFIG.write_text(json.dumps(asdict(s),indent=2)+'\n',encoding='utf-8')
                self.log.appendPlainText('Đã lưu hardware_settings.json.')
            if self.ready:
                self.backend.request('settings',settings=s)
                self.backend.request('arm',enabled=True)
        except Exception as e: self.log.appendPlainText(str(e))
    def load_settings(self):
        if not CONFIG.exists(): return
        try:
            s=Settings(**json.loads(CONFIG.read_text(encoding='utf-8')))
            for widget,value in [(self.open,s.open_deg),(self.closed,s.closed_deg),(self.speed,s.speed_rpm),(self.close_a,s.close_a),(self.hold_a,s.hold_a),(self.contact_a,s.contact_a),(self.preload,s.preload_deg),(self.timeout,s.move_timeout_s)]:
                if value is not None: widget.setValue(value)
            self.log.appendPlainText('Đã nạp góc OPEN/CLOSED và thông số động cơ đã lưu.')
        except Exception as e: self.log.appendPlainText('Load settings: '+str(e))
    def enable_connected_controls(self):
        # Run once per successful connection, never from periodic status updates.
        try:
            s=self.settings()
            self.backend.request('settings',settings=s)
            self.backend.request('arm',enabled=True)
        except ValueError as exc:
            self.log.appendPlainText('Đã kết nối. Nhập góc OPEN/CLOSED hợp lệ rồi bấm Save settings: '+str(exc))
        self.backend.request('udp',enabled=True)
    def stop(self): self.backend.stop_requested.set()
    def poll(self):
        latest=None
        for _ in range(100):
            try: kind,data=self.events.get_nowait()
            except queue.Empty: break
            if kind=='log':
                self.log.appendPlainText(data['message'])
            elif kind=='diagnostic': self.show_diagnostic(data['report'])
            elif kind=='status': latest=data
            elif kind=='connection':
                self._connecting=False; self.connect_btn.setEnabled(not data['connected'])
                if data['connected']: self.enable_connected_controls()
            elif kind=='finished': self.log.appendPlainText('Backend stopped')
        if latest:
            self.mode.setEnabled(not latest['connected'] and not self._connecting)
            self.active=latest['active']; self.pending=latest['pending']; self.fb=latest['fb']; self.ready=latest['connected']
            target_text='—' if latest['target'] is None else f"{latest['target']:.3f} deg"
            self.overview.setText(f"{'MÔ PHỎNG' if self.mode.currentData()=='simulation' else 'ĐỘNG CƠ THẬT'} · {'Đã kết nối' if self.ready else 'Chưa kết nối'} · {latest['state']}" + (f" · {latest['error']}" if latest['error'] else ''))
            control_text='Sẵn sàng điều khiển' if latest['armed'] else 'Điều khiển đang dừng'
            udp_text='UDP VR5 :5050 → :5051' if latest['udp'] else 'UDP chưa bật'
            self.overview.setText(self.overview.text()+f' · {control_text} · {udp_text}')
            self.state.setText(f"{latest['state']} | {'CONNECTED' if self.ready else 'DISCONNECTED'} | {control_text} | {udp_text} | Target {target_text} | I_limit {latest['limit']:.1f} A | {latest['error']}")
            if self.fb:
                cur=self.fb.get('current_a'); temp=self.fb.get('temperature_c')
                self.feedback.setText(f"Position {self.fb['position']:.3f} deg | Current {cur if cur is not None else '—'} A | Temperature {temp if temp is not None else '—'} C | Feedback age {latest['feedback_age']:.2f} s")
                self.trace.points.append(cur); self.trace.update()
            self.jaws.stale=not self.ready or latest['feedback_age']>.5
            if not self.fb: self.feedback.setText('Position — | Current — | Temperature —')
            self.jaws.position=self.fb['position'] if self.fb else None
            self.update_aperture(latest)
            self.jaws.target=latest['target']; self.jaws.open_deg=self.open.value(); self.jaws.closed_deg=self.closed.value()
            self.jaws.mode=self.mode.currentData(); self.jaws.object_present=False; self.jaws.state=latest['state']; self.jaws.update()
            for widget in self.config_controls: widget.setEnabled(not self.active and not latest['armed'])
            for b in self.motion_buttons: b.setEnabled(latest['armed'] and self.ready and not self.pending)
        self.update_connection_buttons()
    def closeEvent(self,event):
        self.backend.stop_requested.set(); self.backend.quit_requested.set(); self.backend.join(timeout=3)
        if self.backend.is_alive():
            self.log.appendPlainText('Waiting for serial operation to end…'); event.ignore(); return
        self.timer.stop(); self.port_timer.stop(); self.monitor.close(); event.accept()

if __name__=='__main__':
    app=QApplication(sys.argv); app.setStyle('Fusion'); win=Window(); win.show(); sys.exit(app.exec())






