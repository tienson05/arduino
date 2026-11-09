# Arduino Uno – Điều khiển 2 động cơ DC có encoder

Firmware cho **Arduino Uno**, viết bằng framework Arduino và quản lý bằng **PlatformIO**.

## Tính năng

- Điều khiển 2 động cơ DC (chiều quay và tốc độ).
- Đọc encoder của từng động cơ ở chế độ **x4** (đếm cả sườn lên và sườn xuống của cả hai kênh A và B), cho độ phân giải gấp 4 lần số xung/vòng danh định của encoder.

## Yêu cầu

**Phần cứng**

- Arduino Uno (ATmega328P) và cáp USB
- 2 động cơ DC có encoder 2 kênh (A/B)
- Mạch driver động cơ
- Nguồn cấp riêng cho động cơ (nối chung GND với Arduino)

**Phần mềm**

- [Visual Studio Code](https://code.visualstudio.com/)
- Extension [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) cho VS Code

## Cài đặt và nạp chương trình

1. Cài đặt **VS Code**.
2. Mở tab **Extensions**, tìm và cài **PlatformIO IDE**. Đợi quá trình cài đặt hoàn tất, sau đó khởi động lại VS Code nếu được yêu cầu.
3. Nhấn vào biểu tượng **PlatformIO** (hình đầu con kiến) ở thanh bên trái, chọn **Open Project** và mở thư mục chứa dự án này (thư mục có file `platformio.ini`).
4. Kiểm tra file `platformio.ini`, đảm bảo môi trường build dành cho Arduino Uno, ví dụ:

   ```ini
   [env:uno]
   platform = atmelavr
   board = uno
   framework = arduino
   ; upload_port = COM3   ; bỏ comment và sửa nếu không tự nhận cổng
   ```

5. Kiểm tra lại cấu hình chân trong mã nguồn cho khớp với mạch thực tế.
6. Kết nối Arduino Uno với máy tính qua cáp USB. PlatformIO thường tự nhận cổng; nếu không, chỉ định `upload_port` trong `platformio.ini`.
7. Nạp code cho Arduino


### Gửi dữ liệu từ arduino lên pi
1. Cách tìm đúng tên cổng của Arduino trên Pi: chạy lệnh **ls /dev/tty** trong terminal, tìm cổng có tiền tố **ttyACM** hoặc **ttyUSB**