# EC Gripper Qt

Ứng dụng desktop Python + Qt/PySide6 điều khiển tay kẹp EC-A4310-P2-36 qua Waveshare USB-CAN-A, kèm mô phỏng và hình tay kẹp 2D ngay trên GUI.

![GUI](preview.png)

## Tính năng

- Simulation: thử Open / Close / Move, vật cản giả lập, phát hiện tiếp xúc và giữ dòng; không cần USB.
- Real motor: driver EC-A4310 đã dùng trong project cũ, góc encoder thật điều khiển hình hiển thị.
- Chọn cổng kèm mô tả thiết bị; mặc định ẩn /dev/ttyS*, có tùy chọn hiện tất cả.
- Hiệu chuẩn hai đầu hành trình; vị trí yêu cầu 0–1000 được ánh xạ thành góc motor.
- Current threshold khi đóng/mở và giữ, tốc độ, preload, phản hồi nhiệt độ/dòng/vị trí.
- UDP tương thích VR5: cổng lệnh 5050, phản hồi hoàn thành 5051.
- Worker riêng cho CAN/UDP, GUI cập nhật khoảng 20 Hz.

## Phạm vi Simulink

Chưa có file .slx nguồn nên bản này chưa chuyển hoặc nhúng mô hình Simulink. Simulation hiện là mô hình Python đơn giản. MATLAB có thể gửi lệnh qua `matlab/gripper_command.m`; đây là UDP client chứ không phải Simulink model hoặc vòng điều khiển realtime. Muốn dùng đúng mô hình .slx, cần xác định đầu vào/ra, chu kỳ mẫu và toolbox của mô hình đó. Không cần MATLAB để chạy GUI này.

## Cài đặt

Yêu cầu Python 3.10 trở lên, Windows hoặc Ubuntu 22.04.

Ubuntu, mở terminal tại thư mục ec-gripper-qt:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Nếu chưa có venv: `sudo apt install python3-venv`. Nếu Qt thiếu xcb-cursor: `sudo apt install libxcb-cursor0`.

Windows, mở Command Prompt tại thư mục ec-gripper-qt:

```bat
py -m venv .venv
.venv\Scripts\activate.bat
python -m pip install -r requirements.txt
python main.py
```

## Chạy mô phỏng trước

1. Giữ chế độ **Simulation — no USB required**.
2. Nhấn **Connect**. Góc giả lập ban đầu 0°, demo calibration 0–90°.
3. Tích **ARM** rồi nhấn **CLOSE**. Khi bật **Simulated object present**, tay kẹp gặp vật ở 55% hành trình, chuyển HOLDING, current threshold giảm xuống I_hold.
4. Nhấn **OPEN**: hình tay kẹp mở theo phản hồi mô hình.
5. Bỏ **Simulated object present** rồi Close để thấy tay kẹp tới hết mục tiêu.
6. Có thể nhập 700 và nhấn Move target; target=63° với demo 0–90°.

Hình 2D là sơ đồ tỷ lệ, không phải CAD hoặc phép đo khe mm. Mô hình synthetic dùng giới hạn tốc độ và vật cản, chưa mô phỏng điện/cơ đầy đủ và chưa dự đoán lực N. Dòng/nhiệt trong Simulation là giả lập. Trạng thái HOLDING chỉ xác nhận logic giữ, chưa có thuật toán xác nhận vật trượt/rơi.

## Điều khiển EC thật

1. Disconnect trước khi đổi mode. Chọn **Real EC-A4310 — USB-CAN**.
2. Chọn cổng có tên thiết bị Waveshare/USB Serial/CH340 tương ứng. Linux thường /dev/ttyUSB0; Windows COMx. Có thể nhập đường dẫn cổng trực tiếp. Danh sách liệt kê thiết bị hệ điều hành nhận được, không chứng minh cổng đang rảnh.
3. Connect chỉ cấu hình adapter, hỏi ID, đặt Q&A và đọc góc. Chờ 5.2 giây trước chuyển động.
4. Xác định **Open angle** và **Closed angle** trong hành trình cơ khí đã kiểm chứng. Có nút lấy góc hiện tại; không tự chạy tìm cữ. Không dùng góc demo 0/90 làm hiệu chuẩn robot thật.
5. Tích xác nhận hiệu chuẩn, Save settings. Cấu hình thật nằm trong hardware_settings.json và không được đưa lên GitHub. Mỗi phiên phải xác nhận lại tọa độ, nhất là nếu mất nguồn làm thay đổi gốc encoder.
6. ARM, thử thay đổi góc nhỏ bằng Move target trước khi chạy hết hành trình.
7. OPEN/CLOSE trên GUI gửi trực tiếp qua worker và CAN; không cần robot_main. Hình tay kẹp lấy góc encoder phản hồi, không chạy hoạt ảnh giả để thay thế feedback thật.

Driver gốc gửi servo-position 8 byte: position, speed, current threshold, ack=2. Dòng có độ phân giải 0.1 A theo driver cũ. Vòng servo nằm trong EC-A4310; Python không phát PWM/FOC.

Mặc định thử: tốc độ 2 rpm ở demo, I_close=0.5 A, I_hold=0.3 A, contact=0.4 A. Khi sang motor thật, kiểm tra/chỉnh lại theo cơ cấu. Giới hạn 3 A là cap của ứng dụng cũ, không đồng nghĩa dòng an toàn cho mọi tay kẹp. Không đổi dòng sang N khi chưa hiệu chuẩn cơ cấu/cảm biến.

**STOP** ngừng stream servo-position và dựa vào heartbeat motor khoảng 500 ms theo driver cũ. Đây không phải lệnh phanh/disable tức thì được xác minh; có thể làm mất lực giữ. Đỡ vật trước STOP/Disconnect/đóng cửa sổ. Chưa thử phần cứng EC thật trong môi trường tạo mã.

## UDP VR5 và MATLAB

Bật **VR5 UDP** khi đã Connect và ARM. Tắt simulator cũ để tránh tranh cổng 5050.

| Nội dung | Chiều | Cổng |
|---|---|---:|
| `GRIPPER_OPEN` | robot_main/MATLAB → Qt | 5050 |
| `GRIPPER_CLOSE:700` | robot_main/MATLAB → Qt | 5050 |
| `GRIPPER_COMPLETE` | Qt → bên điều khiển | 5051 |
| `GRIPPER_ABORTED` | Qt → bên điều khiển | 5051 |

Dữ liệu ASCII, một lệnh/một datagram, không thêm ký tự xuống dòng. Chỉ một nguồn gửi lệnh tuần tự. Không chạy MATLAB client hoặc tools/send_udp.py cùng robot_main vì chúng cùng cần nghe 5051. Chỉ localhost (127.0.0.1) được cấu hình trong phiên bản này.

Lực/dòng/tốc độ chưa có trường UDP trong giao thức gốc: đặt trên GUI trước ARM. COMPLETE phát khi tới target ổn định hoặc chạm vật và giữ ổn định; heartbeat CAN tiếp tục sau COMPLETE. ABORTED khi đang pending và bị STOP/lỗi/timeout. Sau COMPLETE nếu xuất hiện lỗi mới, GUI báo lỗi và dừng, nhưng giao thức VR5 chưa có thông báo lỗi bất đồng bộ được chuẩn hóa.

Thử bằng Python khi Qt đang Simulation + ARM + UDP:

```bash
python tools/send_udp.py close --position 700
python tools/send_udp.py open
```

Thử bằng MATLAB có Instrument Control Toolbox và hàm udpport:

```matlab
addpath('matlab');
gripper_command('close',700)
gripper_command('open')
```

`matlab/gripper_command.m` đã viết theo API MathWorks, chưa chạy trong MATLAB tại đây. Cần file .slx của bạn để kết nối các block/signal cụ thể. Timeout client không đồng nghĩa motor đã dừng; kiểm tra Qt và STOP khi cần, không tự gửi lại lệnh.

## Lỗi cổng Ubuntu

Nếu dmesg báo ch341 attached ttyUSB0 rồi brltty ngắt cổng: khi không dùng thiết bị Braille, có thể gỡ `sudo apt remove brltty`, rút/cắm lại adapter, Refresh ports. Nếu báo Permission denied: `sudo usermod -aG dialout "$USER"`, đăng xuất/đăng nhập lại. Nếu không có USB trong danh sách, kiểm tra `lsusb` và log `sudo dmesg -w`. Cài lại Qt không giải quyết thiết bị USB bị ngắt ở tầng hệ điều hành.

## Cấu trúc mã

| File | Vai trò |
|---|---|
| main.py | Cửa sổ Qt, nút bấm, chọn cổng/mode, calibration |
| visual.py | Hình tay kẹp 2D theo góc phản hồi |
| backend.py | Worker CAN/UDP, kết nối, timeout, heartbeat |
| control.py | State machine, ánh xạ 0–1000, detection và hold |
| simulation.py | Motor giả lập để chạy không USB |
| ec_a4310_waveshare_test.py | Driver gốc từ project EC của bạn |
| matlab/gripper_command.m | Gửi lệnh UDP từ MATLAB |
| tools/send_udp.py | Client UDP Python để thử |
| GITHUB_VI.md | Tạo repository và push mã |

## Kiểm thử

```bash
python -m unittest discover -s tests -v
```

Đã qua 9 kiểm tra: parser, mapping hai chiều, contact/hold, fault/timeout, native CAN payload, UDP mock, heartbeat/STOP và chu trình simulation close/hold/open. GUI đã được chạy offscreen với nút Connect/ARM và hình HOLDING, rồi đóng worker. Chưa chạy trên phần cứng EC thật; chưa kiểm thử MATLAB/Simulink.

GitHub Actions có sẵn tại `.github/workflows/tests.yml` để chạy bộ kiểm tra sau push.

## Nguồn

- Driver và nguyên lý contact/hold kế thừa ec_a4310_waveshare_test.py và ec_a4310_grip_hold_v3.py của project đã cung cấp.
- MATLAB UDP: https://www.mathworks.com/help/instrument/udpport.html
- GitHub import: https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github

Chưa đặt giấy phép phân phối vì quyền phân phối driver gốc cần do chủ repository xác định.
# ec-gripper-qt
# ec-gripper-qt


## USB port diagnostics update

- The port picker now stays accessible in Simulation. The mode banner and Connect simulation / Connect hardware labels identify which connection is used.
- Refresh scans pyserial and Linux ttyUSB/ttyACM nodes plus by-id/by-path aliases. CH340/CH341 VID:PID 1a86:7523 is highlighted as a candidate, not proof of Waveshare identity.
- Unfocused/disconnected lists refresh every two seconds. A typed path is preserved. Scanning never opens a serial port or sends commands.
- **Diagnose USB → Copy report** collects visible ports, permissions, USB inventory and relevant kernel messages. Restricted kernel logs are reported honestly; it never runs sudo or changes drivers.
- CLI alternative, inside the venv: `python tools/diagnose_ports.py`. Add `--port /dev/ttyUSB0` to check a selected device node.
- If Simulation was already connected, Disconnect before selecting Real EC-A4310. Missing ttyUSB nodes require resolving the Linux USB/driver conflict; a GUI change cannot recreate them.
