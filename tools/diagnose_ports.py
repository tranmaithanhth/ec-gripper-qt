"""Run in the app venv: python tools/diagnose_ports.py [--port /dev/ttyUSB0]."""
import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ports import diagnose
p=argparse.ArgumentParser();p.add_argument('--port',default='');a=p.parse_args()
print(diagnose(a.port))
