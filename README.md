# Cheesy Integrated Firmware (STM32F411CEU6)

Dự án STM32 tiêu chuẩn (chỉ gồm `Core/Inc`, `Core/Src`, `Drivers/`) — gộp sẵn tầng thấp của bạn bạn và tầng cao tự hành (`EKF` + `3× VL53L0X` + `A*` + `Pure Pursuit` + `FSM`).

## Bước 1 — Tải về & nạp luôn
1. Clone hoặc tải ZIP repo này về máy.
2. Cắm mạch nạp **ST-Link V2** (`SWDIO=PA13`, `SWCLK=PA14`, `GND`, `3V3`), nhấc bánh xe khỏi mặt bàn.
3. Bấm đúp file **`NAP_CODE.bat`** — tự biên dịch và nạp thẳng vào STM32F411CEU6 tại `0x08000000`.

## Bước 2 — Chỉnh thông số ở đâu
- **Chỉnh cố định trong code:** Mở duy nhất file **`Core/Inc/robot_config.h`**:
  - Tốc độ mong muốn từng ô (`10..30 cm/s`): `V_TARGET_O1_CMS` .. `V_TARGET_O5_CMS`
  - Cơ khí bánh xe / encoder: `ROBOT_WHEEL_DIAM_CM`, `ROBOT_TRACK_WIDTH_CM`, `ROBOT_ENCODER_PPR`
  - Tầm quét ToF (bật từ Ô 3): `FSM_SCAN_TRIGGER_X_CM`, `TOF_SCAN_MAX_MM`
  ➔ Sau khi sửa xong, bấm đúp lại **`NAP_CODE.bat`** để nạp.
- **Chỉnh nhanh trên sân qua Bluetooth HC-05 (`9600 baud`):**
  - `VSEG:0,22` ➔ đặt tốc độ cả 5 ô thành `22 cm/s`
  - `VSEG:3,28` ➔ chỉnh riêng Ô 3 thành `28 cm/s` (`1..5` ứng với Ô 1..Ô 5)
- **Chỉnh PI động cơ (`g_m0_kp/ki`, `g_m1_kp/ki`) hoặc đảo chiều cảm biến:** Mở **`Core/Src/app.c`**.

## Bước 3 — Chạy xe trên sân
1. Đặt xe đứng yên trên vạch xuất phát **5 giây đầu** sau khi bật nguồn để MPU6050 hiệu chuẩn gyro.
2. Nhấn nút **`PA11`** trên mạch để xe tự chạy (từ Ô 1 ➔ Ô 2 ➔ vào **Ô 3 tự bật 3× VL53L0X quét né vật cản** ➔ về đích Ô 5).
3. Về đích hoặc dừng xong, nhấn **`PA11`** lần nữa để reset chạy lượt mới (vẫn giữ tốc độ vừa chỉnh qua BLE).
