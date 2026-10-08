# Đưa EC Gripper Qt lên GitHub

## 1. Tạo repository trống

Đăng nhập https://github.com/new, đặt Repository name là `ec-gripper-qt`. Chọn Private hoặc Public theo nhu cầu. Để các lựa chọn khởi tạo README, .gitignore, license trống vì thư mục đã có README và .gitignore. Nhấn Create repository.

## 2. Khởi tạo Git tại thư mục mã

Ubuntu:

```bash
sudo apt install git
cd ~/Desktop/ec-gripper-qt
git init -b main
```

Windows: cài Git for Windows nếu chưa có, mở Git Bash tại thư mục mã và chạy `git init -b main`.

Nếu Git chưa biết tên/email, đặt trong repository này (thay bằng thông tin của bạn, có thể dùng GitHub noreply email):

```bash
git config user.name "YOUR NAME"
git config user.email "YOUR EMAIL"
```

Kiểm tra file sẽ đưa lên:

```bash
git status --short
```

`.venv/`, `__pycache__/`, hardware_settings.json và .env đã nằm trong .gitignore. Không commit cấu hình/mật khẩu riêng. Không đưa nguyên file ZIP lên thay cho source.

## 3. Commit và push

```bash
git add .
git commit -m "Add Qt EC gripper controller and simulation"
git remote add origin https://github.com/YOUR_USERNAME/ec-gripper-qt.git
git push -u origin main
```

Thay YOUR_USERNAME bằng tài khoản GitHub. Dùng chính URL repository vừa tạo. GitHub yêu cầu xác thực Git qua credential manager/token hoặc SSH; không dùng mật khẩu tài khoản thông thường cho Git HTTPS. Không dán token vào code hoặc URL origin. Nếu dùng GitHub CLI đã cài, có thể chạy `gh auth login` và làm theo trình duyệt trước khi push.

## 4. Những lần cập nhật sau

```bash
git add .
git commit -m "Improve gripper control"
git push
```

Tab Actions sẽ hiện kết quả tests. Tests dùng motor giả; pass không xác nhận kết nối phần cứng.

## 5. Tải mã về máy khác

```bash
git clone https://github.com/YOUR_USERNAME/ec-gripper-qt.git
cd ec-gripper-qt
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

Trên Windows dùng `py -m venv .venv` và `.venv\Scripts\activate.bat` trong Command Prompt.

Nguồn: https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github
