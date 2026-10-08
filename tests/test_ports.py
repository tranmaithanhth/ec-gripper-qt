import sys, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ports import discover,Port

class PortTests(unittest.TestCase):
    def test_fallback_without_usb_serial_number_and_alias_dedup(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'ttyUSB0').touch();(root/'serial/by-path').mkdir(parents=True)
            (root/'serial/by-path/usb-device').symlink_to(root/'ttyUSB0')
            with patch('ports.list_ports.comports',return_value=[]):
                ports,errors=discover(dev_root=root)
            self.assertEqual(len(ports),1);self.assertEqual(len(ports[0].aliases),1);self.assertFalse(errors)
    def test_filters_only_legacy_ttyS_not_usb(self):
        infos=[SimpleNamespace(device=d,description='test',vid=v,pid=p,hwid='') for d,v,p in [('/dev/ttyS0',None,None),('/dev/ttyUSB0',0x1a86,0x7523),('COM7',0x1a86,0x7523)]]
        with tempfile.TemporaryDirectory() as tmp,patch('ports.list_ports.comports',return_value=infos):
            ports,_=discover(dev_root=tmp)
            self.assertEqual({p.device for p in ports},{'/dev/ttyUSB0','COM7'})
            self.assertIn('candidate',ports[0].label)
    def test_enumeration_failure_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp)/'ttyACM0').touch()
            with patch('ports.list_ports.comports',side_effect=OSError('test failure')):
                ports,errors=discover(dev_root=tmp)
            self.assertEqual(len(ports),1);self.assertIn('test failure',errors[0])
if __name__=='__main__':unittest.main()
