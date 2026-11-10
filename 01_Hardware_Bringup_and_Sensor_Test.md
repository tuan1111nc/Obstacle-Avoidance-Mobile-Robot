# HƯỚNG DẪN BRING-UP PHẦN CỨNG & KIỂM THỬ NGOẠI VI (HARDWARE TEST)

* **Dự án:** Xe đua tự hành dò mê cung (`xe_dua`)
* **Vi điều khiển:** STM32F401RCT6 (Cortex-M4F @ 84 MHz)
* **Nguồn đối chiếu:** Trích xuất trực tiếp từ file `xe_dua.kicad_sch`

---

## 1. Bảng Sơ Đồ Chân Chính Xác 100% (Theo KiCad Schematic)

> **Lưu ý quan trọng:** Tuyệt đối sử dụng bảng phân bổ chân dưới đây (đã trích xuất chuẩn từ `xe_dua.kicad_sch`).

| Khối chức năng | Chân STM32 | Tên Net (KiCad) | Cấu hình ngoại vi CubeMX | Ghi chú phần cứng |
| :--- | :---: | :--- | :--- | :--- |
| **Cảm biến IR 0..3** | `PA0`..`PA3` | `IR0`..`IR3` | `ADC1_IN0` .. `ADC1_IN3` | Đọc liên tục qua DMA2 Stream0 |
| **PWM Motor Trái** | `PA6`, `PB5` | `pwr1`, `pwr2` | `TIM3_CH1`, `TIM3_CH2` | Điều khiển `AIN1`, `AIN2` (DRV8833) |
| **PWM Motor Phải** | `PB0`, `PB1` | `pwr3`, `pwr4` | `TIM3_CH3`, `TIM3_CH4` | Điều khiển `BIN1`, `BIN2` (DRV8833) |
| **Encoder Phải** | `PA8`, `PA9` | `Encoder_A/B_right`| `TIM1_CH1`, `TIM1_CH2` | Hardware Encoder Mode TI1 & TI2 |
| **Encoder Trái** | `PA15`, `PB3`| `Encoder_A/B_left` | `TIM2_CH1`, `TIM2_CH2` | Hardware Encoder Mode TI1 & TI2 |
| **I2C1 (IMU + ToF)**| `PB8`, `PB9` | `SCL`, `SDA` | `I2C1_SCL`, `I2C1_SDA` | Fast Mode 400 kHz |
| **XSHUT ToF 1..5** | `PC8`..`PC12`| `xshut1`..`xshut5` | `GPIO_Output` | Phân trang địa chỉ I2C cho 5 VL53L0X |
| **Bluetooth HC-06** | `PC6`, `PC7` | `USART6_TX/RX` | `USART6` (115200 / 9600) | Giao tiếp không dây với Laptop |
| **UART1 Debug** | `PB6`, `PB7` | `TX/RX_USART1` | `USART1` (115200) | Ra Header J9 để in log debug |
| **LED Trạng thái** | `PC0`..`PC3` | `LED0`..`LED3` | `GPIO_Output` | Tích cực mức cao (Active High) |
| **Nút START** | `PB12` | `start` | `GPIO_Input` (`Pull-up`) | Nhấn xuống nối GND (Active Low) |
| **Gạt Manual/Auto** | `PB13` | `manual/auto` | `GPIO_Input` (`Pull-up`) | Chọn chế độ chạy |
| **DIP Switch 2-bit**| `PB14`, `PB15`| `bit1`, `bit2` | `GPIO_Input` (`Pull-up`) | Chọn cấu hình / bài test |

---

## 2. Quy Trình Kiểm Tra Nguồn Điện (Trước Khi Cắm Chip/Cảm Biến)

1. **Rút tất cả module cắm rời** (5 cảm biến VL53L0X, MPU6050 nếu dùng rào cắm) ra khỏi mạch.
2. Cắm pin LiPo 2S ($7.4\text{ V} - 8.4\text{ V}$) vào jack `J3` và bật công tắc nguồn `SW3`.
3. Gạt công tắc chọn nguồn 5V (`SW8`) sang phía **`5v_buck`** (dùng nguồn hạ áp từ module Mini560).
4. Dùng đồng hồ VOM đo điện áp:
   * Giữa chân `GND` và ngõ ra `Mini560`: phải đạt $\approx 5.0\text{ V}$.
   * Giữa chân `GND` và chân `3v3` trên Header nạp ST-Link (`J4`): phải đạt chuẩn **$3.3\text{ V} \pm 0.05\text{ V}$** (từ IC ổn áp LDO `AP2112K-3.3`). LED nguồn `D2` phải sáng.
5. Tắt nguồn, cắm lại 5 module VL53L0X đúng chiều chân (`VIN`, `GND`, `SCL`, `SDA`, `GPIO1`, `XSHUT`).

---

## 3. Hướng Dẫn Cấu Hình STM32CubeMX Chi Tiết

Mở **STM32CubeIDE / STM32CubeMX**, chọn chip **`STM32F401RCTx`** và thiết lập lần lượt:

### Bước 3.1: System Core (SYS & RCC)
* **SYS:** Chọn `Debug = Serial Wire` (Bắt buộc để giải phóng chân `PA15` và `PB3` cho Encoder Trái `TIM2`).
* **RCC:** Chọn `High Speed Clock (HSE) = Crystal/Ceramic Resonator`.
* **Clock Configuration:** Nhập tần số thạch anh ngoài (thường là `8 MHz` hoặc `25 MHz` tùy linh kiện hàn ở `Y1`), bật PLL và gõ `84` vào ô **HCLK (MHz)** rồi nhấn Enter để hệ thống chạy tối đa $84\text{ MHz}$ (`APB1 Timer = 84 MHz`, `APB2 Timer = 84 MHz`).

### Bước 3.2: Cấu hình Timer cho Encoder và PWM
* **TIM1 (Encoder Bánh Phải):**
  * Pinout: Click chọn `PA8` $\rightarrow$ `TIM1_CH1`, `PA9` $\rightarrow$ `TIM1_CH2`.
  * Mode: `Combined Channels = Encoder Mode`.
  * Parameter Settings: `Counter Period = 65535`, `Encoder Mode = Encoder Mode TI1 and TI2`.
* **TIM2 (Encoder Bánh Trái):**
  * Pinout: Click chọn `PA15` $\rightarrow$ `TIM2_CH1`, `PB3` $\rightarrow$ `TIM2_CH2`.
  * Mode: `Combined Channels = Encoder Mode`.
  * Parameter Settings: `Counter Period = 4294967295` (32-bit), `Encoder Mode = Encoder Mode TI1 and TI2`.
* **TIM3 (PWM 20 kHz cho DRV8833):**
  * Pinout: Chọn **đúng** các chân: `PA6` (`TIM3_CH1`), `PB5` (`TIM3_CH2`), `PB0` (`TIM3_CH3`), `PB1` (`TIM3_CH4`).
  * Mode: `Channel1..4 = PWM Generation CH1..4`.
  * Parameter Settings: `Prescaler = 0`, `Counter Period (ARR) = 4199` ($\frac{84\text{ MHz}}{4200} = 20\text{ kHz}$).
* **TIM5 (Ngắt điều khiển 1 kHz):**
  * Mode: Check chọn `Internal Clock`.
  * Parameter Settings: `Prescaler = 83` ($84\text{ MHz} / 84 = 1\text{ MHz}$), `Counter Period = 999` ($1\text{ MHz} / 1000 = 1\text{ kHz}$).
  * NVIC Settings: Tích chọn `TIM5 global interrupt`.

### Bước 3.3: Cấu hình ADC1 (4 Cảm biến IR)
* Pinout: Chọn `PA0`..`PA3` làm `ADC1_IN0`..`IN3`.
* Parameter Settings:
  * `Scan Conversion Mode = Enabled`
  * `Continuous Conversion Mode = Enabled`
  * `DMA Continuous Requests = Enabled`
  * `Number Of Conversion = 4`. Cấu hình Rank 1..4 lần lượt là `Channel 0..3`, `Sampling Time = 84 Cycles`.
* DMA Settings: Nhấn `Add` $\rightarrow$ `ADC1` (DMA2 Stream 0), chọn `Mode = Circular`, `Data Width = Half Word` (16-bit).

### Bước 3.4: Giao tiếp I2C1 và UART
* **I2C1:** Chọn chân `PB8` (`I2C1_SCL`) và `PB9` (`I2C1_SDA`). Trong Parameter Settings, chọn `I2C Speed Mode = Fast Mode` ($400\text{ kHz}$).
* **USART6 (Bluetooth):** Chọn `PC6` (`USART6_TX`), `PC7` (`USART6_RX`). `Mode = Asynchronous`, Baudrate = `9600` (mặc định HC-06 mới mua) hoặc `115200` (nếu đã cấu hình AT). Bật `USART6 global interrupt` trong NVIC.
* **USART1 (Debug J9):** Chọn `PB6` (`USART1_TX`), `PB7` (`USART1_RX`). `Mode = Asynchronous`, Baudrate = `115200`.

### Bước 3.5: Cấu hình GPIO
* **Outputs:** `PC0`..`PC3` (Đặt nhãn `LED0`..`LED3`), `PC8`..`PC12` (Đặt nhãn `XSHUT1`..`XSHUT5`).
* **Inputs (Pull-up):** `PB12` (`BTN_START`), `PB13` (`SW_MODE`), `PB14` (`DIP_BIT1`), `PB15` (`DIP_BIT2`). **Bắt buộc** vào mục `GPIO` $\rightarrow$ chọn cả 4 chân `PB12..PB15` và chỉnh `GPIO Pull-up/Pull-down = Pull-up`.

---

## 4. Code Kiểm Thử Toàn Diện Phần Cứng (`bringup_test.c`)

Sau khi nhấn **Generate Code** từ CubeMX, bạn thêm định nghĩa chân dưới đây vào `main.h` và gọi hàm test trong `main.c`. Chương trình sử dụng **DIP Switch 2-bit (`PB14`, `PB15`)** để chọn 4 bài test độc lập:

* **`DIP = 00`**: Test Nút bấm, Công tắc, nháy 4 LED và gửi "Hello" qua Bluetooth/UART.
* **`DIP = 01`**: Test đọc 4 cảm biến dò vạch IR (ADC DMA) và đếm xung 2 bánh Encoder khi quay tay.
* **`DIP = 10`**: Test Bus I2C1, tự động gán địa chỉ `0x30..0x34` cho 5 cảm biến ToF VL53L0X và đọc `WHO_AM_I` + góc Gyro Z từ MPU6050.
* **`DIP = 11`**: Test cầu H DRV8833 và chiều quay 2 động cơ khi nhấn nút `START`.

### 4.1. Định nghĩa chân chuẩn trong `Core/Inc/main.h`

```c
/* ==================== PIN DEFINITIONS (CHUẨN KICAD SCHEMATIC) ==================== */
// 4 LED trạng thái (PC0 - PC3)
#define LED0_PIN            GPIO_PIN_0
#define LED1_PIN            GPIO_PIN_1
#define LED2_PIN            GPIO_PIN_2
#define LED3_PIN            GPIO_PIN_3
#define LED_PORT            GPIOC

// 5 Chân XSHUT của VL53L0X (PC8 - PC12)
#define XSHUT1_PIN          GPIO_PIN_8
#define XSHUT2_PIN          GPIO_PIN_9
#define XSHUT3_PIN          GPIO_PIN_10
#define XSHUT4_PIN          GPIO_PIN_11
#define XSHUT5_PIN          GPIO_PIN_12
#define XSHUT_PORT          GPIOC

// Nút bấm & Công tắc (PB12 - PB15, Active Low - có Pull-up)
#define BTN_START_PIN       GPIO_PIN_12
#define SW_AUTO_MAN_PIN     GPIO_PIN_13
#define DIP_BIT1_PIN        GPIO_PIN_14
#define DIP_BIT2_PIN        GPIO_PIN_15
#define INPUT_PORT          GPIOB
```

### 4.2. Mã nguồn kiểm thử (`Core/Src/bringup_test.c`)

```c
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Khai báo các handle ngoại vi do CubeMX sinh ra trong main.c */
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/* Buffer lưu giá trị ADC 4 kênh IR (0..4095) */
uint16_t ir_adc_raw[4] = {0};

/* Địa chỉ I2C 7-bit (dịch trái 1 bit cho HAL STM32) */
#define MPU6050_I2C_ADDR        (0x68 << 1)
#define VL53L0X_DEFAULT_ADDR    (0x29 << 1)
static const uint8_t TOF_NEW_ADDR[5] = {
    (0x30 << 1), (0x31 << 1), (0x32 << 1), (0x33 << 1), (0x34 << 1)
};
static const uint16_t XSHUT_PINS[5] = {
    XSHUT1_PIN, XSHUT2_PIN, XSHUT3_PIN, XSHUT4_PIN, XSHUT5_PIN
};

/* Hàm gửi chuỗi log đồng thời ra UART1 (Header J9) và UART6 (Bluetooth HC-06) */
void Debug_Print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 50);
}

/* Đọc giá trị 2-bit từ DIP Switch (0..3) */
uint8_t Read_DIP_Mode(void) {
    uint8_t b1 = (HAL_GPIO_ReadPin(INPUT_PORT, DIP_BIT1_PIN) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t b2 = (HAL_GPIO_ReadPin(INPUT_PORT, DIP_BIT2_PIN) == GPIO_PIN_RESET) ? 1 : 0;
    return (b2 << 1) | b1;
}

/* Điều khiển động cơ DRV8833: speed từ -1000 đến +1000 (Tương ứng 0..100% PWM) */
void Motor_Set_PWM(int16_t left_speed, int16_t right_speed) {
    // Giới hạn an toàn 70% như thiết kế (ARR = 4199 -> Max = 2940)
    const int16_t MAX_PWM = 2940;
    if (left_speed > MAX_PWM) left_speed = MAX_PWM;
    if (left_speed < -MAX_PWM) left_speed = -MAX_PWM;
    if (right_speed > MAX_PWM) right_speed = MAX_PWM;
    if (right_speed < -MAX_PWM) right_speed = -MAX_PWM;

    // Động cơ Trái: PA6 (TIM3_CH1) & PB5 (TIM3_CH2) - Chế độ Slow Decay
    if (left_speed >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MAX_PWM - left_speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MAX_PWM + left_speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MAX_PWM);
    }

    // Động cơ Phải: PB0 (TIM3_CH3) & PB1 (TIM3_CH4)
    if (right_speed >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MAX_PWM - right_speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_PWM + right_speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MAX_PWM);
    }
}

/* Khởi tạo phân trang địa chỉ 5 cảm biến VL53L0X qua chân XSHUT */
void Bringup_I2C_ToF_And_IMU(void) {
    char buf[128];

    // 1. Kéo toàn bộ 5 chân XSHUT xuống GND để tắt cả 5 con VL53L0X
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_PINS[i], GPIO_PIN_RESET);
    }
    HAL_Delay(20);

    // 2. Bật từng con VL53L0X lên và đổi địa chỉ I2C thông qua thanh ghi 0x8A
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(XSHUT_PORT, XSHUT_PINS[i], GPIO_PIN_SET);
        HAL_Delay(15); // Đợi chip boot xong ở địa chỉ mặc định 0x29

        if (HAL_I2C_IsDeviceReady(&hi2c1, VL53L0X_DEFAULT_ADDR, 3, 20) == HAL_OK) {
            uint8_t new_7bit_addr = TOF_NEW_ADDR[i] >> 1;
            HAL_I2C_Mem_Write(&hi2c1, VL53L0X_DEFAULT_ADDR, 0x8A, I2C_MEMADD_SIZE_8BIT, &new_7bit_addr, 1, 20);
            HAL_Delay(5);
        }

        // Kiểm tra xem cảm biến đã nhận địa chỉ mới chưa (Đọc Model ID tại 0xC0, chuẩn = 0xEE)
        uint8_t model_id = 0;
        if (HAL_I2C_Mem_Read(&hi2c1, TOF_NEW_ADDR[i], 0xC0, I2C_MEMADD_SIZE_8BIT, &model_id, 1, 20) == HAL_OK) {
            snprintf(buf, sizeof(buf), "[OK] ToF %d @ 0x%02X (ModelID=0x%02X)\r\n", i + 1, TOF_NEW_ADDR[i] >> 1, model_id);
        } else {
            snprintf(buf, sizeof(buf), "[ERR] ToF %d KHONG PHAN HOI!\r\n", i + 1);
        }
        Debug_Print(buf);
    }

    // 3. Kiểm tra MPU6050 tại địa chỉ 0x68 (Đọc thanh ghi WHO_AM_I = 0x75)
    uint8_t who_am_i = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 20) == HAL_OK) {
        uint8_t wake_cmd = 0x00; // Đánh thức MPU6050 khỏi chế độ Sleep (Thanh ghi 0x6B)
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake_cmd, 1, 20);
        snprintf(buf, sizeof(buf), "[OK] MPU6050 @ 0x68 (WHO_AM_I=0x%02X)\r\n", who_am_i);
    } else {
        snprintf(buf, sizeof(buf), "[ERR] MPU6050 KHONG PHAN HOI!\r\n");
    }
    Debug_Print(buf);
}

/* ==================== VÒNG LẶP TEST CHÍNH ==================== */
void Run_Hardware_Bringup_Loop(void) {
    char log_buf[160];

    // Khởi động phần cứng Timer & ADC DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ir_adc_raw, 4);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    Motor_Set_PWM(0, 0);

    Debug_Print("\r\n=== BAT DAU KIEM THU PHAN CUNG XE DUA ===\r\n");
    Bringup_I2C_ToF_And_IMU();

    uint8_t led_idx = 0;
    while (1) {
        uint8_t mode = Read_DIP_Mode();
        uint8_t btn_start = (HAL_GPIO_ReadPin(INPUT_PORT, BTN_START_PIN) == GPIO_PIN_RESET);
        uint8_t sw_auto   = (HAL_GPIO_ReadPin(INPUT_PORT, SW_AUTO_MAN_PIN) == GPIO_PIN_RESET);

        if (mode == 0) {
            /* TEST 0: Nháy đuổi 4 LED và kiểm tra nút bấm / công tắc */
            HAL_GPIO_WritePin(LED_PORT, LED0_PIN | LED1_PIN | LED2_PIN | LED3_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_PORT, (LED0_PIN << led_idx), GPIO_PIN_SET);
            led_idx = (led_idx + 1) % 4;

            snprintf(log_buf, sizeof(log_buf), "[MODE 0 - GPIO] START=%d | AUTO_SW=%d | DIP=%d\r\n", btn_start, sw_auto, mode);
            Debug_Print(log_buf);
            HAL_Delay(250);
        }
        else if (mode == 1) {
            /* TEST 1: Kiểm tra 4 cảm biến IR dò vạch và 2 Encoder */
            int16_t enc_right = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
            int32_t enc_left  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

            snprintf(log_buf, sizeof(log_buf),
                     "[MODE 1 - IR & ENC] IR: [%4u, %4u, %4u, %4u] | EncL: %6ld | EncR: %6d\r\n",
                     ir_adc_raw[0], ir_adc_raw[1], ir_adc_raw[2], ir_adc_raw[3], enc_left, enc_right);
            Debug_Print(log_buf);
            HAL_Delay(200);
        }
        else if (mode == 2) {
            /* TEST 2: Đọc Gyro Z từ MPU6050 & Kiểm tra kết nối 5 ToF */
            uint8_t raw_gz[2] = {0};
            int16_t gz = 0;
            if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, 0x47, I2C_MEMADD_SIZE_8BIT, raw_gz, 2, 10) == HAL_OK) {
                gz = (int16_t)((raw_gz[0] << 8) | raw_gz[1]);
            }
            float gyro_dps = (float)gz / 65.5f; // Giả sử cấu hình +-500 dps

            snprintf(log_buf, sizeof(log_buf), "[MODE 2 - I2C] GyroZ Raw=%6d (%.2f deg/s)\r\n", gz, gyro_dps);
            Debug_Print(log_buf);
            HAL_Delay(200);
        }
        else if (mode == 3) {
            /* TEST 3: Kiểm tra Động cơ (Nhấn giữ nút START để quay bánh) */
            if (btn_start) {
                Debug_Print("[MODE 3 - MOTOR] Tien 30% trong 1s...\r\n");
                Motor_Set_PWM(900, 900);
                HAL_Delay(1000);

                Debug_Print("[MODE 3 - MOTOR] Lui 30% trong 1s...\r\n");
                Motor_Set_PWM(-900, -900);
                HAL_Delay(1000);

                Motor_Set_PWM(0, 0);
                Debug_Print("[MODE 3 - MOTOR] Dung.\r\n");
            } else {
                Motor_Set_PWM(0, 0);
            }
            HAL_Delay(100);
        }
    }
}
```

---

## 5. Checklist Đánh Giá Kết Quả Bring-up

1. **Kiểm tra IR (`MODE 1`):** Đặt xe lên nền sàn trắng, 4 giá trị `IR[0..3]` phải nhỏ (ví dụ $< 800$). Đưa xe đè lên băng dính đen hoặc nhấc bổng xe lên khỏi mặt bàn, giá trị ADC phải tăng vọt ($> 2800$).
2. **Kiểm tra Chiều Encoder (`MODE 1`):** Dùng tay đẩy nhẹ xe **tiến về phía trước**: cả `EncL` và `EncR` đều phải **tăng dương**. Nếu 1 bên bị giảm âm, bạn chỉ cần thêm dấu trừ `-` trước biến đọc counter của bên đó trong code (hoặc đổi `Encoder Polarity` trong CubeMX).
3. **Kiểm tra I2C (`MODE 2`):** Cả 5 dòng `[OK] ToF 1..5` phải xuất hiện với `ModelID=0xEE` và `[OK] MPU6050` với `WHO_AM_I=0x68` (hoặc `0x70`/`0x72` nếu là chip MPU6500/9250 đóng mác GY-521).
4. **Kiểm tra Chiều Động cơ (`MODE 3`):** Nhấc bổng xe lên, gạt `DIP = 11` và bấm `START`. Trong 1 giây đầu cả 2 bánh phải quay theo chiều tiến. Nếu bánh nào quay ngược, đảo vị trí 2 kênh `TIM_CHANNEL` của bánh đó trong hàm `Motor_Set_PWM()`.