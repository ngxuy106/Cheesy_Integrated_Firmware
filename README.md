# Cheesy Integrated Firmware (STM32F411CEU6)

Firmware tích hợp hoàn chỉnh cho xe robot tự hành giải sa bàn trên nền tảng **STM32F411CEU6** (`100 MHz`, `512 KB Flash`, `128 KB SRAM`), tách rõ 3 lớp để hai thành viên phát triển độc lập không xung đột code.

## 1. Kiến trúc 3 lớp

| Thư mục | Vai trò | Phụ thuộc STM32 HAL |
|---|---|:-:|
| `Core/` | Driver tầng thấp (`motor_api.c` 100 Hz PID + TIM2/3 Encoder, `imu.c` MPU6050 200 Hz, `vl53l0x.c` 3× ToF, `battery.c`, `ble_telemetry.c`, `app.c`) | Có |
| `Interface/` | Hợp đồng giao tiếp & cầu nối an toàn ngắt (`robot_interface.h`, `robot_hw_bridge.c`) — đọc atomic encoder, kiểm tra độ tươi ToF, slew-rate & giới hạn vận tốc `<= 30 cm/s` | Chỉ `.c` |
| `HighLevel/` | Tầng cao thuần C11: `ekf_localization.c` (EKF 3 trạng thái + ZUPT + chống trượt/treo IMU), `arena_map.c` (C-Space `50×74`), `obstacle_mapper.c` (3× ToF quét từ Ô 3..Ô 5), `astar_planner.c`, `pure_pursuit.c`, `mission_fsm.c` | **Không** |
| `Tests/` | Bộ unit test & mô phỏng vật lý 2D vòng kín chạy trực tiếp trên PC bằng GCC | **Không** |

Xem chi tiết sơ đồ, pinout và quy trình chạy sân trong [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md).

## 2. Biên dịch & Kiểm thử nhanh (Windows PowerShell)

### Chạy bộ test & mô phỏng 2D trên PC (GCC)
```powershell
.\Tests\run_host_tests.ps1
```

### Biên dịch firmware STM32F411CEU6 (CMake + `arm-none-eabi-gcc`)
```powershell
.\build_firmware.ps1 -Preset Release
```
File nạp ra tại `build/Release/cheesy_integrated.{bin,hex,elf}` (nạp tại `0x08000000`).

## 3. 3 cách hiệu chỉnh vận tốc mong muốn `v_target` (`10..30 cm/s`)

1. **Sửa trong code:** Chỉnh `V_TARGET_O1_CMS .. V_TARGET_O5_CMS` trong `HighLevel/Inc/robot_config.h`.
2. **Chỉnh live qua BLE (`USART2` 9600 baud):** Gửi `VSEG:0,<v_cms>` (đặt cả 5 ô) hoặc `VSEG:<1..5>,<v_cms>` (chỉnh riêng Ô 1..Ô 5) — giữ nguyên qua các lần nhấn `PA11` reset.
3. **Chỉnh live qua STM32CubeMonitor:** Sửa trực tiếp mảng `g_v_target_cms[0..4]` trong `Core/Src/app.c`.
