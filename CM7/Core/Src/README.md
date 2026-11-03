# Mã nguồn Cortex-M7

File main.c chứa chương trình điều khiển động cơ trên STM32H755:

- Điều khiển DRV8871 bằng PWM.
- Đọc encoder GA25-370 và tính RPM.
- Điều khiển tốc độ bằng PI.
- Thử nghiệm PWM cố định 30%.
- Gửi dữ liệu UART sang MATLAB.

Đây là mã nguồn thuộc CM7, cần kết hợp với cấu hình CubeMX
và các thư viện của project để biên dịch.
