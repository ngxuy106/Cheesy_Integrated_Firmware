# Cheesy Integrated Firmware (STM32F411CEU6)

Dự án STM32 tiêu chuẩn (chỉ gồm `Core/Inc`, `Core/Src`, `Drivers/`, `icons/`) — gộp sẵn tầng thấp của bạn bạn và tầng cao tự hành (`EKF` + `3× VL53L0X` + `A*` + `Pure Pursuit` + `FSM`).

## 2 công cụ bấm đúp có icon riêng

| File / Shortcut | Công dụng |
|---|---|
| **`CHINH_THONG_SO.bat`** *(hoặc shortcut `CHINH_THONG_SO`)* | Mở **cửa sổ giao diện Windows** chỉnh toàn bộ **17 thông số** (tốc độ Ô 1..Ô 5, `Vmax`, gia tốc, `Kp/Ki`, bánh xe, encoder, tầm quét ToF, đảo chiều IMU...).<br>• Kết nối **Bluetooth HC-05** ➔ bấm **Gửi LIVE qua Bluetooth** ➔ bấm **Lưu vào Flash xe** — **không cần nạp lại code!**<br>• Hoặc bấm **Lưu vào file Code (.h)** rồi **Build & Nạp ST-Link**. |
| **`NAP_CODE.bat`** *(hoặc shortcut `NAP_CODE`)* | Cắm ST-Link V2 (`SWDIO=PA13`, `SWCLK=PA14`, `GND`, `3V3`), nhấc bánh xe ➔ bấm đúp để tự biên dịch và nạp firmware vào `0x08000000`. |

## Chạy xe trên sân
1. Đặt xe đứng yên trên vạch xuất phát **5 giây đầu** sau khi cấp nguồn để MPU6050 hiệu chuẩn gyro.
2. Nhấn nút **`PA11`** trên mạch để xe tự chạy (Ô 1 ➔ Ô 2 ➔ vào **Ô 3 tự bật 3× VL53L0X quét né vật cản** ➔ về đích Ô 5).
3. Về đích hoặc dừng xong, nhấn **`PA11`** lần nữa để reset chạy lượt mới.
