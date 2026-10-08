"""Hardware independent sequencer; servo loops remain inside EC-A4310."""
from dataclasses import dataclass, asdict
from collections import deque
import math

@dataclass(frozen=True)
class Settings:
    open_deg: float | None = None
    closed_deg: float | None = None
    speed_rpm: float = 1.0
    close_a: float = 0.5
    hold_a: float = 0.3
    contact_a: float = 0.4
    preload_deg: float = 1.0
    tolerance_deg: float = 0.25
    move_timeout_s: float = 20.0
    max_temp_c: float = 70.0

    def validate(self):
        values = asdict(self)
        if any(not math.isfinite(v) for v in values.values() if v is not None):
            raise ValueError('All settings must be finite')
        if self.open_deg is None or self.closed_deg is None:
            raise ValueError('Set open and closed angles before motion')
        span = abs(self.closed_deg-self.open_deg)
        if span < 1 or span > 360:
            raise ValueError('Calibration span must be 1–360 deg; confirm the mechanical travel')
        if not 0.1 <= self.speed_rpm <= 5:
            raise ValueError('Test speed must be 0.1–5 rpm')
        if not 0.1 <= self.hold_a <= self.close_a <= 3:
            raise ValueError('Require 0.1 <= I_hold <= I_close <= 3 A (application test cap)')
        if not 0.1 <= self.contact_a <= self.close_a:
            raise ValueError('Contact threshold must be 0.1 A to I_close')
        if not 0 <= self.preload_deg <= min(5,span):
            raise ValueError('Invalid preload')
        if not 0.05 <= self.tolerance_deg <= min(0.5,span/4):
            raise ValueError('Invalid tolerance')
        if not 2 <= self.move_timeout_s <= 120:
            raise ValueError('Timeout must be 2–120 s')
        if not 20 <= self.max_temp_c <= 70:
            raise ValueError('Temperature limit must be 20–70 C')

    def position(self, raw):
        self.validate()
        if not isinstance(raw,int) or not 0<=raw<=1000:
            raise ValueError('Position must be integer 0–1000')
        return self.open_deg+(self.closed_deg-self.open_deg)*raw/1000

class Sequencer:
    def __init__(self):
        self.state='IDLE'; self.active=False; self.target=None
        self.current_limit=0.; self.settings=None; self.history=deque()
        self.last_pos=None; self.last_time=None; self.error=''; self.completion=None

    def start(self, raw, settings, fb, now):
        settings.validate(); self.check_feedback(fb,settings)
        pos=float(fb['position']); lo=min(settings.open_deg,settings.closed_deg); hi=max(settings.open_deg,settings.closed_deg)
        if not lo-settings.tolerance_deg<=pos<=hi+settings.tolerance_deg:
            raise ValueError('Position is outside calibrated travel')
        self.settings=settings; self.target=settings.position(raw); self.raw=raw
        self.direction=1 if settings.closed_deg>settings.open_deg else -1
        self.closing=(self.target-pos)*self.direction > settings.tolerance_deg
        self.start_pos=pos; self.started=now; self.contact_count=0
        self.settled_since=None; self.history.clear(); self.error=''; self.completion=None
        self.last_pos=pos; self.last_time=now; self.current_limit=settings.close_a
        self.active=True; self.state='MOVING'

    @staticmethod
    def check_feedback(fb, settings):
        for key in ('position','current_a','temperature_c'):
            if not math.isfinite(float(fb[key])): raise RuntimeError('Non-finite motor feedback')
        if int(str(fb['error']).split()[0]) != 0: raise RuntimeError('Motor fault: '+str(fb['error']))
        if fb['temperature_c']>=settings.max_temp_c: raise RuntimeError('Motor temperature limit reached')
        # Allow quantization/transient margin; this is supervisory, not a hardware current limiter.
        if abs(fb['current_a'])>settings.close_a+0.3: raise RuntimeError('Current exceeds application threshold')

    def stop(self, reason='Stopped; command stream ended'):
        self.active=False; self.state='STOPPED'; self.error=reason

    def fault(self, reason):
        self.stop(reason); self.state='FAULT'; self.completion='GRIPPER_ABORTED'

    def update(self,fb,now):
        if not self.active: return
        s=self.settings
        try:
            self.check_feedback(fb,s)
            lo=min(s.open_deg,s.closed_deg); hi=max(s.open_deg,s.closed_deg)
            if not lo-s.tolerance_deg<=fb['position']<=hi+s.tolerance_deg:
                raise RuntimeError('Motor passed calibrated travel limit')
        except Exception as e:
            self.fault(str(e)); return
        pos=fb['position']; elapsed=max(1e-6,now-self.last_time)
        speed=abs(pos-self.last_pos)/elapsed
        self.last_pos=pos; self.last_time=now
        self.history.append((now,pos))
        while self.history and now-self.history[0][0]>.30: self.history.popleft()
        if self.state=='MOVING':
            stalled=len(self.history)>=2 and now-self.history[0][0]>=.21 and abs(pos-self.history[0][1])<=.15
            candidate=(self.closing and stalled and abs(fb['current_a'])>=s.contact_a
                and abs(pos-self.start_pos)>=.5 and now-self.started>=.35
                and abs(self.target-pos)>s.tolerance_deg)
            self.contact_count=self.contact_count+1 if candidate else 0
            if self.contact_count>=3:
                # Never advance beyond the requested target or calibrated endpoints.
                proposed=pos+self.direction*s.preload_deg
                self.target=min(self.target,proposed) if self.direction>0 else max(self.target,proposed)
                self.current_limit=s.hold_a; self.state='SETTLING_HOLD'; self.settled_since=None
                self.hold_started=now
                return
            good=abs(self.target-pos)<=s.tolerance_deg and speed<=1.0
            self.settled_since=(self.settled_since or now) if good else None
            if self.settled_since is not None and now-self.settled_since>=.2:
                self.state='AT_POSITION'; self.completion='GRIPPER_COMPLETE'
            elif now-self.started>s.move_timeout_s:
                self.fault('Motion timeout')
        elif self.state=='SETTLING_HOLD':
            # Evaluate feedback AFTER reduced-current commands have been sent.
            good=speed<=1.0 and abs(fb['current_a'])<=s.hold_a+.3
            self.settled_since=(self.settled_since or now) if good else None
            if self.settled_since is not None and now-self.settled_since>=.3:
                self.state='HOLDING'; self.completion='GRIPPER_COMPLETE'
            elif now-self.hold_started>3:
                self.fault('Hold did not settle')
        # HOLDING / AT_POSITION keep heartbeat until next move, STOP or disconnect.

    def take_completion(self):
        value=self.completion; self.completion=None; return value

def parse_vr5(text):
    if text=='GRIPPER_OPEN': return 0
    prefix='GRIPPER_CLOSE:'
    if text.startswith(prefix):
        payload=text[len(prefix):]
        if payload.isascii() and payload.isdigit() and 0<=int(payload)<=1000: return int(payload)
    raise ValueError('Expected GRIPPER_OPEN or GRIPPER_CLOSE:<0..1000>')
