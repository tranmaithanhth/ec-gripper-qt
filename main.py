import sys, json, queue, time, threading
from pathlib import Path
from dataclasses import asdict
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPainter, QColor, QPen
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QVBoxLayout,QHBoxLayout,
 QFormLayout,QDoubleSpinBox,QSpinBox,QPushButton,QLabel,QComboBox,QCheckBox,QPlainTextEdit,QGroupBox,QDialog)
from ports import discover, diagnose
from control import Settings
from backend import Backend
from visual import JawView
from collections import deque

CONFIG=Path(__file__).with_name('hardware_settings.json')

class Trace(QWidget):
    def __init__(self):
        super().__init__(); self.points=deque(maxlen=500); self.setMinimumHeight(140)
    def paintEvent(self,event):
        p=QPainter(self); p.fillRect(self.rect(),QColor('#102234'))
        p.setPen(QColor('#d4e2ee')); p.drawText(15,22,'Actual current (A), −3 … +3; last 25 s nominal')
        p.setPen(QColor('#456074')); mid=(self.height()+30)/2
        p.drawLine(15,int(mid),self.width()-15,int(mid))
        p.setPen(QPen(QColor('#38d6b4'),2)); data=list(self.points)
        for i in range(1,len(data)):
            if data[i-1] is None or data[i] is None: continue
            y1=mid-data[i-1]/3*(self.height()-50)/2; y2=mid-data[i]/3*(self.height()-50)/2
            p.drawLine(int(15+(i-1)*(self.width()-30)/500),int(y1),int(15+i*(self.width()-30)/500),int(y2))

class Window(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('EC Gripper Qt • Simulation / Real motor'); self.resize(1080,980)
        self.events=queue.Queue(); self.backend=Backend(self.events); self.backend.start()
        self._ports_initialized=False; self._connecting=False; self._port_signature=None; self._diagnosing=False
        self.fb=None; self.ready=False; self.active=False; self.pending=False; self.sync=False
        self.setStyleSheet('QGroupBox {font-weight:bold;} QPushButton {padding:7px;}')
        root=QWidget(); self.setCentralWidget(root); layout=QVBoxLayout(root)
        layout.addWidget(QLabel('EC-A4310-P2-36 + Waveshare USB-CAN-A | servo position + current threshold'))
        mode_row=QHBoxLayout(); layout.addLayout(mode_row)
        self.mode=QComboBox(); self.mode.addItem('Simulation — no USB required','simulation'); self.mode.addItem('Real EC-A4310 — USB-CAN','real'); mode_row.addWidget(self.mode)
        self.object=QCheckBox('Simulated object present'); self.object.setChecked(True); self.object.toggled.connect(lambda on:self.backend.request('sim_object',present=on)); mode_row.addWidget(self.object)
        self.allports=QCheckBox('Show ttyS / all ports'); self.allports.toggled.connect(lambda _:self.refresh_ports()); mode_row.addWidget(self.allports)
        self.jaws=JawView(); layout.addWidget(self.jaws)
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
        self.calibrated=QCheckBox('Both angles verified within mechanical travel'); c.addRow(self.calibrated)
        params=QGroupBox('Current thresholds — not force in N'); groups.addWidget(params); f=QFormLayout(params)
        self.speed=self.spin(f,'Speed (rpm)',.1,5,1,1)
        self.close_a=self.spin(f,'I_close / I_open max (A)',.1,3,.5,1)
        self.hold_a=self.spin(f,'I_hold threshold (A)',.1,3,.3,1)
        self.contact_a=self.spin(f,'Contact detection (A)',.1,3,.4,1)
        self.preload=self.spin(f,'Preload (deg)',0,5,1,2)
        self.timeout=self.spin(f,'Move timeout (s)',2,120,20,1)
        row=QHBoxLayout(); layout.addLayout(row)
        self.save=QPushButton('Save settings'); self.save.clicked.connect(self.save_settings); row.addWidget(self.save)
        self.arm=QCheckBox('ARM — permit manual / VR5 motion'); self.arm.toggled.connect(self.arm_changed); row.addWidget(self.arm)
        self.udp=QCheckBox('VR5 UDP :5050 → :5051'); self.udp.toggled.connect(lambda on:self.backend.request('udp',enabled=on) if not self.sync else None); row.addWidget(self.udp)
        self.motion_buttons=[]; row=QHBoxLayout(); layout.addLayout(row)
        self.raw=QSpinBox(); self.raw.setRange(0,1000); self.raw.setValue(700); row.addWidget(self.raw)
        for label,raw in [('Move target',None),('OPEN',0),('CLOSE',1000)]:
            b=QPushButton(label); b.clicked.connect(lambda checked=False,value=raw:self.backend.request('move',raw=self.raw.value() if value is None else value)); row.addWidget(b); self.motion_buttons.append(b)
        self.stop_btn=QPushButton('STOP — cease command stream'); self.stop_btn.setStyleSheet('background:#bd3030;color:white;font-weight:bold;'); self.stop_btn.clicked.connect(self.stop); row.addWidget(self.stop_btn)
        layout.addWidget(QLabel('STOP relies on existing motor heartbeat timeout (~500 ms); support payload before stopping.'))
        self.state=QLabel('DISCONNECTED'); layout.addWidget(self.state)
        self.feedback=QLabel('Position — | Current — | Temperature —'); layout.addWidget(self.feedback)
        self.trace=Trace(); layout.addWidget(self.trace)
        self.log=QPlainTextEdit(); self.log.setReadOnly(True); self.log.setMaximumBlockCount(600); layout.addWidget(self.log)
        self.config_controls=[self.open,self.closed,self.open_set,self.close_set,self.calibrated,self.speed,self.close_a,self.hold_a,self.contact_a,self.preload,self.timeout,self.save]
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.mode_changed()
        self.timer=QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(50)
        self.port_timer=QTimer(self); self.port_timer.timeout.connect(self.auto_refresh_ports); self.port_timer.start(2000)
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
    def connect_motor(self):
        if self.mode.currentData()=='real' and not self.port.currentText().strip(): self.log.appendPlainText('Select USB-CAN serial port'); return
        self._connecting=True; self.mode.setEnabled(False); self.connect_btn.setEnabled(False); self.backend.request('connect',port=self.port.currentText().split(' | ',1)[0].strip(),motor_id=self.mid.value() or None,mode=self.mode.currentData())
        self.backend.request('sim_object',present=self.object.isChecked())
    def mode_changed(self):
        sim=self.mode.currentData()=='simulation'
        self.object.setEnabled(sim); self.port.setEnabled(True); self.mid.setEnabled(not sim)
        self.connect_btn.setText('Connect simulation' if sim else 'Connect hardware')
        self.refresh_ports()
        self.jaws.mode=self.mode.currentData(); self.jaws.update()
        if sim:
            self.open.setValue(0); self.closed.setValue(90); self.calibrated.setChecked(True)
            self.speed.setValue(2); self.close_a.setValue(.5); self.hold_a.setValue(.3); self.contact_a.setValue(.4)
        else:
            self.open.setValue(0); self.closed.setValue(0)
            self.load_settings()
            self.calibrated.setChecked(False)
            self.log.appendPlainText('REAL MODE: verify mechanical calibration before ARM; demo angles are not hardware calibration.')
    def capture(self,widget):
        if self.fb and self.backend.can and time.monotonic()-self.backend.fb_time<.5:
            widget.setValue(self.fb['position'])
        else: self.log.appendPlainText('Connect and obtain fresh position first')
    def settings(self):
        if not self.calibrated.isChecked(): raise ValueError('Verify OPEN/CLOSED calibration first')
        s=Settings(open_deg=self.open.value(),closed_deg=self.closed.value(),speed_rpm=self.speed.value(),close_a=self.close_a.value(),hold_a=self.hold_a.value(),contact_a=self.contact_a.value(),preload_deg=self.preload.value(),move_timeout_s=self.timeout.value())
        s.validate(); return s
    def save_settings(self):
        try:
            if self.mode.currentData()=='simulation':
                self.log.appendPlainText('Simulation uses demo calibration; hardware settings were not overwritten'); return
            s=self.settings(); CONFIG.write_text(json.dumps(asdict(s),indent=2)+'\n',encoding='utf-8'); self.log.appendPlainText('Saved hardware_settings.json; Arm remains an explicit action each session')
        except Exception as e: self.log.appendPlainText(str(e))
    def load_settings(self):
        if not CONFIG.exists(): return
        try:
            s=Settings(**json.loads(CONFIG.read_text(encoding='utf-8')))
            for widget,value in [(self.open,s.open_deg),(self.closed,s.closed_deg),(self.speed,s.speed_rpm),(self.close_a,s.close_a),(self.hold_a,s.hold_a),(self.contact_a,s.contact_a),(self.preload,s.preload_deg),(self.timeout,s.move_timeout_s)]:
                if value is not None: widget.setValue(value)
            # User rechecks calibration after each restart (encoder origin may change).
            self.log.appendPlainText('Settings loaded. Recheck calibration against current motor angle.')
        except Exception as e: self.log.appendPlainText('Load settings: '+str(e))
    def arm_changed(self,on):
        if self.sync: return
        try:
            if on:
                s=self.settings(); self.backend.request('settings',settings=s)
            self.backend.request('arm',enabled=on)
        except Exception as e:
            self.log.appendPlainText(str(e)); self.sync=True; self.arm.setChecked(False); self.sync=False
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
            elif kind=='finished': self.log.appendPlainText('Backend stopped')
        if latest:
            self.mode.setEnabled(not latest['connected'] and not self._connecting); self.object.setEnabled(self.mode.currentData()=='simulation')
            self.active=latest['active']; self.pending=latest['pending']; self.fb=latest['fb']; self.ready=latest['connected']
            self.sync=True; self.arm.setChecked(latest['armed']); self.udp.setChecked(latest['udp']); self.sync=False
            target_text='—' if latest['target'] is None else f"{latest['target']:.3f} deg"
            self.state.setText(f"{latest['state']} | {'CONNECTED' if self.ready else 'DISCONNECTED'} | {'ARMED' if latest['armed'] else 'DISARMED'} | Target {target_text} | I_limit {latest['limit']:.1f} A | {latest['error']}")
            if self.fb:
                cur=self.fb.get('current_a'); temp=self.fb.get('temperature_c')
                self.feedback.setText(f"Position {self.fb['position']:.3f} deg | Current {cur if cur is not None else '—'} A | Temperature {temp if temp is not None else '—'} C | Feedback age {latest['feedback_age']:.2f} s")
                self.trace.points.append(cur); self.trace.update()
            self.jaws.position=self.fb['position'] if self.fb else None
            self.jaws.target=latest['target']; self.jaws.open_deg=self.open.value(); self.jaws.closed_deg=self.closed.value()
            self.jaws.mode=self.mode.currentData(); self.jaws.object_present=self.object.isChecked(); self.jaws.state=latest['state']; self.jaws.update()
            for widget in self.config_controls: widget.setEnabled(not self.active and not latest['armed'])
            for b in self.motion_buttons: b.setEnabled(latest['armed'] and self.ready and not self.pending)
            self.connect_btn.setEnabled(not self.ready and not self._connecting)
    def closeEvent(self,event):
        self.backend.stop_requested.set(); self.backend.quit_requested.set(); self.backend.join(timeout=3)
        if self.backend.is_alive():
            self.log.appendPlainText('Waiting for serial operation to end…'); event.ignore(); return
        self.timer.stop(); self.port_timer.stop(); event.accept()

if __name__=='__main__':
    app=QApplication(sys.argv); app.setStyle('Fusion'); win=Window(); win.show(); sys.exit(app.exec())

