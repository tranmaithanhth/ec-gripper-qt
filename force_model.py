"""Quasi-static contact-normal estimates, not measured load or lift capacity."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class ForceCalibration:
    # Output-shaft Nm per amp of the actual CAN feedback current. Zero = unknown.
    torque_per_amp: float = 0.
    efficiency: float = .70
    v_angle_deg: float = 149.5

    def validate(self):
        if not all(math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError('Thông số lực phải hữu hạn.')
        if not 0 <= self.torque_per_amp <= 1000 or not 0 < self.efficiency <= 1 or not 1 <= self.v_angle_deg <= 180:
            raise ValueError('Kiểm tra K (0…1000 Nm/A), hiệu suất (0…1] và góc V (1…180°).')

def estimate_forces(geometry, position, encoder_open, encoder_closed, current_a, calibration):
    """Return sum of normal-force magnitudes: two flat or four V contacts.

    D is the full gap. Virtual work gives F_jaw = eta*tau/abs(dD/dq).
    Encoder q and output-shaft torque must be a conjugate angle/torque pair.
    The existing affine encoder-to-crank calibration supplies dtheta/dq.
    """
    calibration.validate()
    if calibration.torque_per_amp <= 0:
        raise ValueError('Chưa nhập hệ số mô-men trục ra / dòng phản hồi (Nm/A).')
    if current_a is None or not math.isfinite(current_a):
        raise ValueError('Chưa có dòng điện phản hồi hợp lệ.')
    geometry.width(position, encoder_open, encoder_closed)  # validates calibration/domain
    fraction=(position-encoder_open)/(encoder_closed-encoder_open)
    t=math.radians(geometry.crank_open_deg+fraction*(geometry.crank_closed_deg-geometry.crank_open_deg))
    g=geometry.inputs
    u=g.e_mm-g.r_mm*math.sin(t)
    root=math.sqrt(g.l_mm*g.l_mm-u*u)
    # Reject ill-conditioned configurations near either kinematic singularity.
    if root < 1e-6*max(g.l_mm, 1.):
        raise ValueError('Gần điểm kỳ dị cơ khí; không ước lượng lực.')
    dx=-g.r_mm*math.sin(t)+g.r_mm*math.cos(t)*u/root
    ratio=(geometry.crank_closed_deg-geometry.crank_open_deg)/(encoder_closed-encoder_open)
    jacobian=abs(2*dx*ratio)/1000.  # m/rad of output encoder
    if jacobian < 1e-5:
        raise ValueError('Gần điểm chết cơ khí; không ước lượng lực.')
    flat=2*calibration.efficiency*calibration.torque_per_amp*abs(current_a)/jacobian
    v=flat/math.sin(math.radians(calibration.v_angle_deg/2))
    if not all(math.isfinite(value) for value in (flat,v)):
        raise ValueError('Kết quả lực không hữu hạn.')
    return flat,v
