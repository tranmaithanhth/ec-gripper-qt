from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtCore import QRectF, Qt

class JawView(QWidget):
    """Schematic jaw aperture derived from encoder angle; not a calibrated CAD model."""
    def __init__(self):
        super().__init__();self.setMinimumHeight(220)
        self.position=None;self.target=None;self.open_deg=0.;self.closed_deg=90.
        self.mode='simulation';self.object_present=True;self.state='DISCONNECTED'
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing);p.fillRect(self.rect(),QColor('#102234'))
        p.setPen(QColor('#e2eef6'));p.setFont(QFont('Sans',10))
        p.drawText(16,24,'SIMULATION / synthetic feedback' if self.mode=='simulation' else 'REAL MOTOR / encoder feedback')
        p.drawText(16,45,'Schematic aperture from calibrated angles; actual jaw width is not measured')
        span=self.closed_deg-self.open_deg if self.closed_deg is not None and self.open_deg is not None else 0
        valid=self.position is not None and abs(span)>1e-6
        raw=max(0.,min(1.,(self.position-self.open_deg)/span)) if valid else 0
        cx=self.width()/2; base_y=75.; maxgap=min(240,self.width()*.42)
        gap=60+(1-raw)*maxgap
        p.fillRect(QRectF(cx-maxgap/2-30,base_y,maxgap+60,35),QColor('#427b9b'))
        for sign in (-1,1):
            x=cx+sign*gap/2
            p.fillRect(QRectF(x-10,base_y+35,20,75),QColor('#35c9bd'))
            p.fillRect(QRectF(x-10 if sign<0 else x-30,base_y+95,40,16),QColor('#35c9bd'))
        if self.mode=='simulation' and self.object_present:
            width=.45*maxgap
            p.fillRect(QRectF(cx-width/2,base_y+67,width,45),QColor('#e9b45d'))
        p.setPen(QColor('#e2eef6'))
        pos=f'{self.position:.3f} deg' if self.position is not None else '—'
        p.drawText(16,self.height()-18,f'{self.state} | Encoder {pos} | Closure {raw*1000:.0f}/1000' if valid else f'{self.state} | Encoder {pos} | Calibration required')
