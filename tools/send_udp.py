"""Standalone VR5 protocol client. Do not run while robot_main owns UDP 5051."""
import argparse, socket
p=argparse.ArgumentParser();p.add_argument('command',choices=['open','close']);p.add_argument('--position',type=int,default=700);p.add_argument('--timeout',type=float,default=25)
a=p.parse_args()
if not 0<=a.position<=1000:p.error('position must be 0..1000')
msg='GRIPPER_OPEN' if a.command=='open' else f'GRIPPER_CLOSE:{a.position}'
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
    s.bind(('127.0.0.1',5051));s.settimeout(a.timeout);s.sendto(msg.encode('ascii'),('127.0.0.1',5050))
    try: print(s.recvfrom(256)[0].decode('ascii'))
    except socket.timeout: raise SystemExit('No response. Command may still be active; check Qt GUI. No automatic retry.')
