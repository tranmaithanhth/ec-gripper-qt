"""Read-only serial discovery/diagnostics. Never opens or writes to a port."""
import os, sys, platform, subprocess
from pathlib import Path
from dataclasses import dataclass, field
from serial.tools import list_ports

@dataclass
class Port:
    device: str
    description: str = 'Serial device'
    vid: int | None = None
    pid: int | None = None
    hwid: str = ''
    aliases: list = field(default_factory=list)
    @property
    def usb(self):
        return self.vid is not None or self.device.startswith(('/dev/ttyUSB','/dev/ttyACM'))
    @property
    def label(self):
        chip=' [CH340/CH341 candidate]' if (self.vid,self.pid)==(0x1a86,0x7523) else ''
        access=''
        if os.name=='posix' and Path(self.device).exists() and not os.access(self.device,os.R_OK|os.W_OK):
            access=' [no read/write permission]'
        return f'{self.device} | {self.description}{chip}{access}'

def discover(show_all=False, dev_root=None):
    root=Path(dev_root or '/dev'); found={}; errors=[]
    try:
        for info in list_ports.comports():
            device=info.device
            if not show_all and device.startswith('/dev/ttyS'): continue
            found[os.path.realpath(device)]=Port(device,info.description or 'Serial device',info.vid,info.pid,info.hwid)
    except Exception as e: errors.append(f'pyserial enumeration: {e}')
    if sys.platform.startswith('linux') or dev_root is not None:
        # Fallback when pyserial omits nodes, including adapters without USB serial number.
        for pattern in ('ttyUSB*','ttyACM*','serial/by-id/*','serial/by-path/*'):
            for path in sorted(root.glob(pattern)):
                if not path.exists(): continue
                canonical=os.path.realpath(path)
                entry=found.setdefault(canonical,Port(canonical,'USB serial (device-node fallback)'))
                if str(path)!=canonical and str(path) not in entry.aliases: entry.aliases.append(str(path))
    return sorted(found.values(),key=lambda p:(not p.usb,p.device)), errors

def run_readonly(args):
    try:
        r=subprocess.run(args,capture_output=True,text=True,timeout=3,check=False)
        return (r.stdout+r.stderr).strip() or f'exit={r.returncode}; no output'
    except (OSError,subprocess.TimeoutExpired) as e: return str(e)

def diagnose(selected=''):
    ports,errors=discover(show_all=True)
    lines=['EC Gripper USB diagnostics (read-only; no serial port opened)',
           f'OS: {platform.platform()}',f'Python: {sys.executable}',f'Selected: {selected or "none"}',
           'Listed does not guarantee the port is idle or that CAN motor communication works.']
    if errors: lines.extend(errors)
    for p in ports:
        lines.extend([p.label,f'  HWID: {p.hwid}; aliases: {", ".join(p.aliases) or "none"}'])
    usb=[p for p in ports if p.usb]
    if not usb and sys.platform.startswith('linux'):
        lines.append('No ttyUSB/ttyACM USB serial endpoint found. Qt cannot create a missing kernel device.')
    if sys.platform.startswith('linux'):
        try:
            import grp
            groups=', '.join(grp.getgrgid(g).gr_name for g in os.getgroups())
        except (KeyError,OSError): groups='unknown'
        lines.append('Current process groups: '+groups)
        if selected:
            lines.append(f'Exists: {Path(selected).exists()}; read/write access: {os.access(selected,os.R_OK|os.W_OK)}')
            # fuser output is observational. An empty result does not guarantee availability.
            if selected.startswith('/dev/') and Path(selected).exists():
                lines.extend(['Processes visible through fuser:',run_readonly(['fuser','-v',selected])])
        lines.extend(['USB inventory:',run_readonly(['lsusb'])])
        log=run_readonly(['dmesg','--color=never'])
        relevant=[line for line in log.splitlines() if any(s in line.lower() for s in ('ch341','ch340','ttyusb','ttyacm','brltty','permission','permitted','denied'))]
        lines.extend(['Recent relevant kernel messages (may include older unplug events):', '\n'.join(relevant[-35:]) or 'No relevant messages readable.'])
        lines.extend(['If dmesg is restricted, run sudo dmesg -w, replug the adapter, then copy only the new lines.',
                      'If ch341 attaches then brltty disconnects it, resolve that USB conflict; replug and Refresh ports.',
                      'If permission is missing: sudo usermod -aG dialout "$USER", then log out/in and restart the app.',
                      'A missing /dev/serial/by-id folder alone does not mean missing driver; some adapters have no serial number.'])
    return '\n'.join(lines)
