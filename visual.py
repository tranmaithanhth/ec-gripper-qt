"""Responsive schematic driven by feedback; geometry is not a CAD measurement."""
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QLinearGradient
from PySide6.QtCore import QRectF, Qt

class JawView(QWidget):
    def __init__(self):
        super().__init__(); self.setMinimumSize(320,300)
        self.position=None; self.target=None; self.open_deg=0.; self.closed_deg=90.
        self.mode='simulation'; self.object_present=True; self.state='DISCONNECTED'; self.stale=True

    def paintEvent(self,event):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#121f2c'))
        p.setPen(QPen(QColor('#1c2e3e'),1))
        for x in range(0,self.width(),28): p.drawLine(x,0,x,self.height())
        for y in range(0,self.height(),28): p.drawLine(0,y,self.width(),y)
        p.setFont(QFont('Sans',10)); p.setPen(QColor('#9cb2c5'))
        p.drawText(18,26,'MÔ PHỎNG' if self.mode=='simulation' else 'PHẢN HỒI ENCODER')
        p.drawText(18,47,'Sơ đồ 2D · độ mở quy đổi từ góc, chưa hiệu chuẩn theo mm')
        span=self.closed_deg-self.open_deg
        valid=self.position is not None and abs(span)>1e-6
        closure=max(0.,min(1.,(self.position-self.open_deg)/span)) if valid else 0.
        p.save()
        scale=min((self.width()-40)/600,(self.height()-95)/340)
        p.translate(self.width()/2,58+(self.height()-95-340*scale)/2); p.scale(scale,scale)
        def box(x,y,w,h,top,bottom,r=6):
            gradient=QLinearGradient(x,y,x,y+h); gradient.setColorAt(0,QColor(top)); gradient.setColorAt(1,QColor(bottom))
            p.setPen(QPen(QColor('#526a7d'),1)); p.setBrush(gradient); p.drawRoundedRect(QRectF(x,y,w,h),r,r)
        # Cast shadow, motor housing, guide and independent carriages.
        p.setPen(Qt.NoPen); p.setBrush(QColor(0,0,0,55)); p.drawEllipse(QRectF(-230,298,460,25))
        box(-66,0,132,65,'#536b81','#263a4c',12)
        box(-195,45,390,96,'#adc0cf','#536b80',10)
        box(-172,102,344,21,'#172837','#344c60',4)
        for x in (-176,176):
            for y in (62,85):
                p.setBrush(QColor('#182b3b')); p.setPen(QPen(QColor('#c0d0dc'),1)); p.drawEllipse(QRectF(x-4,y-4,8,8))
        p.setPen(QColor('#dce9f2')); p.setFont(QFont('Sans',12,QFont.Bold)); p.drawText(QRectF(-100,60,200,28),Qt.AlignCenter,'EC · A4310')
        gap=260*(1-closure)
        # Pads meet at zero; at synthetic contact (55%), gap equals object width.
        for side in (-1,1):
            inner=side*gap/2
            x=inner-38 if side<0 else inner
            box(x-8,112,54,39,'#5be0c6','#248477',5)
            box(x,151,38,125,'#b8cad8','#526e84',5)
            pad_x=inner-10 if side<0 else inner
            box(pad_x,202,10,74,'#344d60','#172b3b',2)
            p.setPen(QPen(QColor('#7394a8'),1))
            for y in range(208,273,8): p.drawLine(int(pad_x+2),y,int(pad_x+8),y)
        if self.mode=='simulation' and self.object_present:
            box(-58.5,205,117,71,'#e9ba70','#ac733b',9)
            p.setPen(QColor('#513e27')); p.setFont(QFont('Sans',10)); p.drawText(QRectF(-58,225,116,25),Qt.AlignCenter,'VẬT MÔ PHỎNG')
        p.setPen(QPen(QColor('#61d8c0'),1.5))
        p.drawLine(int(-gap/2),294,int(gap/2),294)
        for x in (-gap/2,gap/2): p.drawLine(int(x),286,int(x),302)
        p.restore()
        p.setFont(QFont('Sans',10)); p.setPen(QColor('#69dfc4') if valid and not self.stale else QColor('#e2b76e'))
        caption=f'Mức đóng {closure*100:.0f}%  ·  Góc {self.position:.2f}°' if valid else 'Chờ phản hồi và hiệu chuẩn góc mở / đóng'
        if valid and self.stale: caption+='  ·  Phản hồi cũ'
        p.drawText(18,self.height()-17,caption)
        p.end()
