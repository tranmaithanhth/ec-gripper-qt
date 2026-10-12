"""Offset slider-crank model. Dimensions mm; encoder/crank angles degrees.

Recovered design relation: x=r*cos(t)+sqrt(l*l-(e-r*sin(t))**2)
D=D_closed+2*(x-x_closed). No linear aperture approximation or auto scaling.
This model is for display only and never modifies motor commands/limits.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Geometry:
    max_mm: float = 80.
    closed_mm: float = 0.
    r_mm: float = 0.
    l_mm: float = 0.
    e_mm: float = 0.
    crank_open_deg: float = 0.
    crank_closed_deg: float = 0.

    def x(self, degrees):
        t=math.radians(degrees)
        radicand=self.l_mm**2-(self.e_mm-self.r_mm*math.sin(t))**2
        if radicand <= 0: raise ValueError('Cơ cấu ngoài miền hình học hoặc tại điểm kỳ dị.')
        return self.r_mm*math.cos(t)+math.sqrt(radicand)

    def at_fraction(self, fraction):
        if not math.isfinite(fraction) or not 0 <= fraction <= 1:
            raise ValueError('Góc nằm ngoài khoảng hiệu chuẩn OPEN/CLOSED.')
        t=self.crank_open_deg+fraction*(self.crank_closed_deg-self.crank_open_deg)
        return self.closed_mm+2*(self.x(t)-self.x(self.crank_closed_deg))

    def validate(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()): raise ValueError('Thông số phải hữu hạn.')
        if not 0 <= self.closed_mm < self.max_mm: raise ValueError('Cần 0 ≤ độ mở đóng < độ mở tối đa.')
        if self.r_mm <= 0 or self.l_mm <= 0: raise ValueError('Nhập r, l và hai góc cơ cấu từ kết quả tối ưu.')
        if abs(self.crank_closed_deg-self.crank_open_deg)<1e-6: raise ValueError('Hai góc cơ cấu không được trùng nhau.')
        # Check extrema of the square-root term over the ENTIRE angular interval.
        lo,hi=sorted(map(math.radians,(self.crank_open_deg,self.crank_closed_deg)))
        extrema=[lo,hi]+[math.pi/2+k*math.pi for k in range(math.ceil((lo-math.pi/2)/math.pi),math.floor((hi-math.pi/2)/math.pi)+1)]
        if any(abs(self.e_mm-self.r_mm*math.sin(t))>=self.l_mm for t in extrema):
            raise ValueError('Hành trình đi qua miền hình học không hợp lệ / kỳ dị.')
        widths=[self.at_fraction(i/1000) for i in range(1001)]
        if abs(widths[0]-self.max_mm)>max(.1,self.max_mm*.001):
            raise ValueError(f'Mô hình cho OPEN={widths[0]:.3f} mm, khác độ mở tối đa {self.max_mm:.3f} mm. Kiểm tra kích thước/góc.')
        if any(b>a+1e-7 for a,b in zip(widths,widths[1:])):
            raise ValueError('Độ mở không giảm đơn điệu từ OPEN tới CLOSED.')
        return self

    def width(self, position, encoder_open, encoder_closed):
        if not all(math.isfinite(v) for v in (position,encoder_open,encoder_closed)) or abs(encoder_closed-encoder_open)<1e-6:
            raise ValueError('Chưa hiệu chuẩn góc encoder OPEN/CLOSED.')
        return self.at_fraction((position-encoder_open)/(encoder_closed-encoder_open))
