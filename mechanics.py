"""Five independent inputs; infer slider-crank OPEN/CLOSED angles for display.

D = doffset + 2*x, x = r*cos(t) + sqrt(l*l-(e-r*sin(t))**2).
No motor command or encoder calibration is derived/changed here.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Geometry:
    max_mm: float = 80.
    r_mm: float = 0.
    l_mm: float = 0.
    e_mm: float = 0.
    doffset_mm: float = 0.

    def check_inputs(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError('Thông số phải là số hữu hạn.')
        if self.max_mm <= 0 or self.r_mm <= 0 or self.l_mm <= 0:
            raise ValueError('Nhập độ mở tối đa, r và l lớn hơn 0. e và Doffset có thể bằng 0 hoặc âm.')

    def x(self, degrees):
        t=math.radians(degrees)
        rad=self.l_mm**2-(self.e_mm-self.r_mm*math.sin(t))**2
        if rad <= 1e-12: raise ValueError('Cơ cấu ngoài miền hình học hoặc tại điểm kỳ dị.')
        return self.r_mm*math.cos(t)+math.sqrt(rad)

    def roots(self, width):
        """Analytic inverse, filtered to the positive square-root assembly branch."""
        x=(width-self.doffset_mm)/2
        radius=math.hypot(x,self.e_mm)
        if radius<1e-12: return []
        ratio=(x*x+self.e_mm**2+self.r_mm**2-self.l_mm**2)/(2*self.r_mm*radius)
        if abs(ratio)>1+1e-10: return []
        alpha=math.acos(max(-1.,min(1.,ratio)))
        phi=math.atan2(self.e_mm,x)
        roots=[]
        for t in (phi+alpha,phi-alpha):
            deg=math.degrees(t)%360
            try: residual=abs(self.doffset_mm+2*self.x(deg)-width)
            except ValueError: continue
            if residual<1e-6 and all(abs((deg-old+180)%360-180)>1e-6 for old in roots): roots.append(deg)
        return sorted(roots)

    def solve(self):
        self.check_inputs()
        opens=self.roots(self.max_mm); closes=self.roots(0.)
        if not opens or not closes:
            missing='OPEN' if not opens else 'CLOSED = 0 mm'
            raise ValueError(f'Không có nghiệm {missing} với bộ số này. D = Doffset + 2x; kiểm tra Doffset/kích thước/độ mở. Các ô nhập vẫn được giữ.')
        solutions=[]
        for a in opens:
            for b in closes:
                for winding in (-1,0,1):
                    end=b+360*winding
                    if not 1e-6<abs(end-a)<=360: continue
                    model=SolvedGeometry(self,a,end)
                    try: model.check_path()
                    except ValueError: continue
                    solutions.append(model)
        if not solutions: raise ValueError('Không có hành trình OPEN → CLOSED giảm đơn điệu và hợp lệ. Các ô nhập vẫn được giữ.')
        # Prefer increasing crank angle; expose alternatives instead of hiding ambiguity.
        return sorted(solutions,key=lambda s:(s.crank_closed_deg<s.crank_open_deg,abs(s.crank_closed_deg-s.crank_open_deg),s.crank_open_deg))

@dataclass(frozen=True)
class SolvedGeometry:
    inputs: Geometry
    crank_open_deg: float
    crank_closed_deg: float

    @property
    def max_mm(self): return self.inputs.max_mm

    def at_fraction(self,fraction):
        if not math.isfinite(fraction) or not 0<=fraction<=1:
            raise ValueError('Góc nằm ngoài khoảng hiệu chuẩn OPEN/CLOSED.')
        theta=self.crank_open_deg+fraction*(self.crank_closed_deg-self.crank_open_deg)
        width=self.inputs.doffset_mm+2*self.inputs.x(theta)
        return 0. if abs(width)<1e-8 else width

    def check_path(self):
        lo,hi=sorted(map(math.radians,(self.crank_open_deg,self.crank_closed_deg)))
        extrema=[lo,hi]+[math.pi/2+k*math.pi for k in range(math.ceil((lo-math.pi/2)/math.pi),math.floor((hi-math.pi/2)/math.pi)+1)]
        g=self.inputs
        if any(abs(g.e_mm-g.r_mm*math.sin(t))>=g.l_mm for t in extrema): raise ValueError('Miền căn không hợp lệ.')
        # Include exact stationary points to avoid missing a short reversal near an endpoint.
        knots=[i/720 for i in range(721)]
        for length in (g.r_mm+g.l_mm,g.r_mm-g.l_mm):
            if abs(length)<1e-12 or abs(g.e_mm/length)>1: continue
            a=math.asin(g.e_mm/length)
            for root in (a,math.pi-a):
                for k in range(-3,4):
                    t=root+2*k*math.pi
                    if lo<t<hi:
                        knots.append((math.degrees(t)-self.crank_open_deg)/(self.crank_closed_deg-self.crank_open_deg))
        widths=[self.at_fraction(s) for s in sorted(knots)]
        if any(b>a+1e-7 for a,b in zip(widths,widths[1:])): raise ValueError('Độ mở không đơn điệu.')

    def width(self,position,encoder_open,encoder_closed):
        if not all(math.isfinite(v) for v in (position,encoder_open,encoder_closed)) or abs(encoder_closed-encoder_open)<1e-6:
            raise ValueError('Chưa hiệu chuẩn góc encoder OPEN/CLOSED.')
        return self.at_fraction((position-encoder_open)/(encoder_closed-encoder_open))


def read_config(data):
    """Return inputs and preferred branch angles; migrate previous valid config."""
    if 'doffset_mm' in data:
        inputs=Geometry(**{key:data[key] for key in Geometry.__dataclass_fields__})
        preferred=data.get('selected_angles')
    else:
        if data.get('closed_mm',0)!=0: raise ValueError('Cấu hình cũ có độ mở đóng khác 0 mm; cần chuyển đổi thủ công. File cũ vẫn giữ nguyên.')
        base=Geometry(max_mm=data['max_mm'],r_mm=data['r_mm'],l_mm=data['l_mm'],e_mm=data['e_mm'])
        offset=-2*base.x(data['crank_closed_deg'])
        inputs=Geometry(base.max_mm,base.r_mm,base.l_mm,base.e_mm,offset)
        preferred=[data['crank_open_deg'],data['crank_closed_deg']]
    if not all(math.isfinite(v) for v in inputs.__dict__.values()): raise ValueError('Cấu hình chứa số không hữu hạn.')
    if preferred is not None and (not isinstance(preferred,list) or len(preferred)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in preferred)):
        raise ValueError('Nhánh cơ cấu đã lưu không hợp lệ.')
    return inputs,preferred
