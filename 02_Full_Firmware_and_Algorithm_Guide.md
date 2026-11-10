# HƯỚNG DẪN KIẾN TRÚC FIRMWARE HOÀN CHỈNH & GIẢI THÍCH THUẬT TOÁN TRÊN STM32

* **Dự án:** Xe đua tự hành dò mê cung (`xe_dua`)
* **Kiến trúc:** Bare-metal `TIM5 1 kHz ISR` + `Main Loop I2C`, tách biệt hoàn toàn `Core` (C thuần) và `Port` (STM32 HAL).

---

## 1. Giải Thích Chi Tiết Cách Thuật Toán Chạy Khi Nạp Vào STM32

Khi bạn nạp chương trình vào chip **STM32F401RCT6**, bộ vi xử lý không chạy tuần tự một mạch từ trên xuống dưới mà hoạt động theo **2 luồng song song** phối hợp chặt chẽ qua cấu trúc dữ liệu `SensorFrame` và `ActuatorCmd`:

### 1.1. Luồng nền (`while(1)` trong `main.c`) – Chuyên xử lý giao tiếp chậm (I2C & UART)
Giao tiếp I2C với `MPU6050` và `5 × VL53L0X` ở tốc độ $400\text{ kHz}$ mất khoảng $0.3\text{ ms} - 1.2\text{ ms}$ mỗi lần đọc. Nếu đặt I2C vào trong ngắt điều khiển sẽ làm trễ nhịp PID bánh xe. Do đó, vòng lặp `while(1)` đảm nhận:
* **Mỗi $2\text{ ms}$ ($500\text{ Hz}$):** Đọc vận tốc góc $\omega_z$ từ thanh ghi Gyro Z của `MPU6050`, trừ đi độ trôi `gyro_bias` (đã đo lúc xe đứng yên 1.5s đầu ở trạng thái `CALIB`) và nhân hệ số thang đo `gyro_scale`.
* **Xoay vòng mỗi $6\text{ ms}$ ($33\text{ Hz}$ mỗi cảm biến ToF):** Kiểm tra xem cảm biến `ToF[i]` đã đo xong chưa (Non-blocking). Khi có kết quả mới (ví dụ cảm biến $i$ báo khoảng cách $z\text{ [m]}$), vòng lặp chính lập tức bật cờ `tof_new_flag[i] = 1` để báo cho bộ lọc EKF ở luồng ngắt biết.

### 1.2. Luồng thời gian thực (`TIM5_IRQHandler` chạy đúng mỗi $1\text{ ms}$ - $1000\text{ Hz}$)
Cứ đúng $1\text{ ms}$, phần cứng Timer 5 kích hoạt ngắt, gọi hàm `robot_step(&sensor_frame, &actuator_cmd)` thực hiện 4 bước:

#### Bước A: Ở mọi chu kỳ $1\text{ ms}$ ($1000\text{ Hz}$) – Điều khiển vận tốc bánh xe (PI + Feedforward)
1. Đọc số xung thay đổi $\Delta N_L, \Delta N_R$ từ phần cứng `TIM2` và `TIM1`.
2. Tính vận tốc thực tế của từng bánh:
   $$v_L = \frac{\Delta N_L \cdot \pi D_{\text{wheel}}}{N_{\text{PPR}} \cdot 0.001}, \quad v_R = \frac{\Delta N_R \cdot \pi D_{\text{wheel}}}{N_{\text{PPR}} \cdot 0.001}$$
3. Bộ điều khiển PI so sánh vận tốc đặt $(v_{L,\text{ref}}, v_{R,\text{ref}})$ với $(v_L, v_R)$ để xuất xung PWM $20\text{ kHz}$ ra `TIM3`:
   $$u = K_{ff} \cdot v_{\text{ref}} + K_p (v_{\text{ref}} - v) + K_i \int (v_{\text{ref}} - v)\,dt$$

#### Bước B: Mỗi $5\text{ ms}$ (`tick % 5 == 0`, tức $200\text{ Hz}$) – Dự đoán vị trí (EKF Predict)
1. Tính quãng đường dịch chuyển trung bình $\Delta s = \frac{v_L + v_R}{2} \cdot \Delta t$ (với $\Delta t = 0.005\text{ s}$) và góc quay $\Delta \theta = \omega_{\text{gyro}} \cdot \Delta t$.
2. **Cập nhật trạng thái dự đoán** $\hat{x}_k = [x, y, \theta]^T$ theo phương trình động học vi sai (Runge-Kutta bậc 2 / Midpoint):
   $$\theta_{\text{mid}} = \theta + \frac{\Delta \theta}{2}$$
   $$x^{-} = x + \Delta s \cos(\theta_{\text{mid}}), \quad y^{-} = y + \Delta s \sin(\theta_{\text{mid}}), \quad \theta^{-} = \text{wrap}(\theta + \Delta \theta)$$
3. **Cập nhật ma trận hiệp phương sai sai số** $P^{-} = F P F^T + Q$, trong đó Jacobian $F$ là:
   $$F = \begin{bmatrix} 1 & 0 & -\Delta s \sin(\theta_{\text{mid}}) \\ 0 & 1 & \Delta s \cos(\theta_{\text{mid}}) \\ 0 & 0 & 1 \end{bmatrix}$$
   Lúc này, độ không chắc chắn $P$ phình to dần theo quãng đường xe đi.

#### Bước C: Khi có số đo ToF mới – Sửa sai số trôi bằng Bản đồ (EKF Update)
1. **Raycast trên STM32:** Từ vị trí dự đoán $(x^{-}, y^{-}, \theta^{-})$, STM32 biết tọa độ và góc gắn của cảm biến `ToF[i]` trên thân xe. Nó "bắn" một tia ảo vào danh sách 17 khối vật cản tĩnh ($10\times 10\text{ cm}$) và 4 bức tường bao của bản đồ để tính ra khoảng cách lý thuyết $h(\hat{x}^{-})$.
2. **Tính ma trận độ nhạy $H$ ($1 \times 3$):** Thay vì giải đạo hàm phức tạp, STM32 dịch thử vị trí đi $\delta = 5\text{ mm}$ và góc $\delta_\theta = 0.5^\circ$ rồi gọi lại hàm Raycast để lấy đạo hàm số:
   $$H = \begin{bmatrix} \frac{h(x+\delta) - h(x)}{\delta} & \frac{h(y+\delta) - h(y)}{\delta} & \frac{h(\theta+\delta_\theta) - h(\theta)}{\delta_\theta} \end{bmatrix}$$
3. **Cổng gác Mahalanobis (Chống nhiễu sượt góc):** Tính sai lệch $\nu = z_{\text{đo}} - h(\hat{x}^{-})$ và phương sai tổng $S = H P^{-} H^T + R$. Nếu:
   $$\frac{\nu^2}{S} > 9.0$$
   thì số đo đó bị tia laser bắn sượt mép khối gỗ hoặc phản xạ sàn $\rightarrow$ **Loại bỏ ngay**, không cho làm hỏng vị trí xe.
4. **Hiệu chỉnh Kalman:** Nếu qua được cổng gác, tính hệ số Kalman $K = P^{-} H^T S^{-1}$ và kéo vị trí xe về đúng thực tế:
   $$\hat{x}^{+} = \hat{x}^{-} + K \nu, \quad P^{+} = (I - K H) P^{-}$$

#### Bước D: Máy trạng thái nhiệm vụ (Mission) & Điều khiển bám quỹ đạo (Motion)
1. **Bám đoạn thẳng $A \rightarrow B$:** Từ tọa độ EKF $(x, y, \theta)$, xe chiếu vuông góc lên đoạn thẳng nối 2 Waypoint $A(x_a, y_a)$ và $B(x_b, y_b)$ để tìm:
   * Sai số ngang $e_y$ (xe lệch trái/phải bao nhiêu mm so với đường chuẩn).
   * Quãng đường còn lại $\text{rem}$ tới điểm $B$.
2. **Luật điều khiển phi tuyến hướng nhìn:**
   $$\psi_{\text{des}} = \psi_{AB} - \arctan(K_y \cdot e_y), \quad \omega_{\text{cmd}} = K_h \cdot \text{wrap}(\psi_{\text{des}} - \theta)$$
   $$v_{\text{cmd}} = \min\left(V_{\max}, \sqrt{2 A_{\text{dec}} \cdot \text{rem}}\right) \cdot \max(0, \cos(e_\theta))$$
3. **Tại điểm quyết định `Q` (Xử lý vật cản ngẫu nhiên Lượt 2):**
   * Khi xe đi hết đoạn đường chung và tới điểm `Q` (trước khu vực có 4 vị trí bốc thăm), máy trạng thái chuyển sang `STATE_OBSERVE`.
   * Xe dừng lại $300\text{ ms}$, đọc các cảm biến ToF phía trước và so sánh với khoảng cách dự kiến của 4 giả thuyết vị trí vật ngẫu nhiên $k \in \{0, 1, 2, 3\}$.
   * Giả thuyết nào có tổng sai số Huber nhỏ nhất sẽ được xác nhận là vị trí có vật cản thật $\rightarrow$ Xe lập tức nạp bảng Waypoint của nhánh né vật cản đó (`Route_R0` .. `Route_R3`) để chạy thẳng về Đích.

---

## 2. Cấu Trúc Thư Mục Dự Án Chuẩn (Tách Lớp Core & Port)

Trong project STM32CubeIDE của bạn, hãy tạo thêm 2 thư mục trong `Core/` hoặc đặt ngang hàng:

```text
STM32_MazeCar_Firmware/
├── Core/
│   ├── Inc/
│   │   ├── main.h                 # Định nghĩa Pinout chuẩn KiCad
│   │   ├── robot_core.h           # Giao diện SensorFrame, ActuatorCmd, EKF, Motion
│   │   └── map_waypoints.h        # Tọa độ 17 vật tĩnh, 4 vị trí ngẫu nhiên & Waypoints
│   └── Src/
│       ├── main.c                 # Khởi tạo HAL, vòng lặp I2C & Ngắt TIM5 1kHz
│       ├── robot_core.c           # Toàn bộ thuật toán C thuần (EKF, Raycast, Motion, Mission)
│       └── stm32_port_drivers.c   # Driver đọc MPU6050, 5x VL53L0X, Bluetooth RC Lượt 1
```

---

## 3. Mã Nguồn Đầy Đủ Cho Xe Đua (`Core` + `Port`)

### 3.1. File Header Giao Diện & Bản Đồ (`Core/Inc/robot_core.h`)

```c
#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include <stdint.h>
#include <stdbool.h>

/* ==================== THÔNG SỐ VẬT LÝ CỦA XE ==================== */
#define WHEEL_RADIUS_M          0.017f      // Bán kính bánh xe N20 (17 mm)
#define WHEEL_BASE_M            0.076f      // Khoảng cách tâm 2 bánh xe (76 mm)
#define ENCODER_PPR             210.0f      // Số xung trên 1 vòng bánh xe (sau hộp số, đếm 4x)
#define METERS_PER_TICK         ((2.0f * 3.14159265f * WHEEL_RADIUS_M) / ENCODER_PPR)

#define NUM_STATIC_OBS          17          // 17 khối vật cản cố định 10x10cm
#define NUM_TOF_SENSORS         5           // 5 cảm biến VL53L0X

/* ==================== CẤU TRÚC DỮ LIỆU GIAO TIẾP CORE <-> PORT ==================== */
typedef struct {
    int32_t  enc_left_ticks;                // Tổng xung Encoder trái
    int32_t  enc_right_ticks;               // Tổng xung Encoder phải
    float    gyro_z_rad_s;                  // Vận tốc góc trục Z (rad/s) từ MPU6050
    float    tof_dist_m[NUM_TOF_SENSORS];   // Khoảng cách 5 ToF (m), < 0 nếu chưa có mẫu mới
    uint16_t ir_adc[4];                     // Giá trị ADC 4 mắt đọc hồng ngoại
    bool     btn_start_pressed;             // Trạng thái nút bấm START
    bool     is_auto_mode;                  // Gạt sang chế độ Tự hành (Lượt 2) hay Lái tay (Lượt 1)
    uint8_t  dip_val;                       // Giá trị 2-bit DIP Switch (Chọn checkpoint khi nhấc xe)
} SensorFrame_t;

typedef struct {
    int16_t  pwm_left;                      // Lệnh PWM bánh trái (-2940 .. +2940)
    int16_t  pwm_right;                     // Lệnh PWM bánh phải (-2940 .. +2940)
    uint8_t  led_mask;                      // Bitmask bật/tắt 4 LED báo trạng thái
} ActuatorCmd_t;

/* ==================== HÌNH HỌC BẢN ĐỒ & WAYPOINT ==================== */
typedef struct {
    float x_min, y_min, x_max, y_max;       // Hộp giới hạn Axis-Aligned Bounding Box (m)
} Box2D_t;

typedef struct {
    float x, y;                             // Tọa độ điểm (m)
} Waypoint_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_CALIB,                            // Đứng yên 1.5s lấy mẫu offset Gyro
    STATE_RUN_COMMON,                       // Chạy từ Xuất phát đến điểm Q
    STATE_OBSERVE,                          // Dừng tại Q quét vật cản ngẫu nhiên
    STATE_RUN_BRANCH,                       // Chạy nhánh R0..R3 về đích
    STATE_LIFTED,                           // Phát hiện bị nhấc xe (Chỉnh xe)
    STATE_FINISH                            // Đã qua vạch đích
} MissionState_t;

/* Khởi tạo và hàm bước thời gian thực 1 kHz */
void RobotCore_Init(void);
void RobotCore_Step_1ms(const SensorFrame_t *in, ActuatorCmd_t *out);
void RobotCore_SetManualCmd(float v_m_s, float w_rad_s);

#endif /* ROBOT_CORE_H */
```

---

### 3.2. File Thuật Toán Lõi C Thuần (`Core/Src/robot_core.c`)

File này chứa toàn bộ thuật toán **EKF 3 trạng thái**, **Raycast hộp 2D**, **Điều khiển bám quỹ đạo (Rotate-then-Drive)**, **Nhận diện vật cản ngẫu nhiên tại điểm Q** và **Bộ điều khiển PI bánh xe 1 kHz**.

```c
#include "robot_core.h"
#include <math.h>
#include <string.h>

/* ==================== 1. DỮ LIỆU BẢN ĐỒ & TUYẾN ĐƯỜNG (CẬP NHẬT TỪ PLANNER.PY) ==================== */
// Kích thước sân: Rộng 1.80 m (trục X), Cao 1.20 m (trục Y)
// Tọa độ 17 vật cản tĩnh (10x10 cm) + Vùng cấm chữ U ở giữa (0.60 <= x <= 1.20, 0.00 <= y <= 0.60)
static const Box2D_t MAP_OBSTACLES[NUM_STATIC_OBS + 1] = {
    {0.60f, 0.00f, 1.20f, 0.60f}, // [0]: Vùng cấm chữ U ở giữa sân
    {0.35f, 1.10f, 0.45f, 1.20f}, // [1]..[17]: 17 khối vật cản tĩnh (Lấy từ layout.py của nhóm)
    {1.10f, 1.10f, 1.20f, 1.20f},
    {0.80f, 0.95f, 0.90f, 1.05f},
    {0.25f, 0.85f, 0.35f, 0.95f},
    {1.20f, 0.80f, 1.30f, 0.90f},
    {0.00f, 0.70f, 0.10f, 0.80f},
    {0.60f, 0.70f, 0.70f, 0.80f},
    {0.70f, 0.60f, 0.80f, 0.70f},
    {0.90f, 0.60f, 1.00f, 0.70f},
    {0.00f, 0.50f, 0.10f, 0.60f},
    {0.50f, 0.40f, 0.60f, 0.50f},
    {1.40f, 0.40f, 1.50f, 0.50f},
    {0.25f, 0.30f, 0.35f, 0.40f},
    {0.50f, 0.20f, 0.60f, 0.30f},
    {1.70f, 0.20f, 1.80f, 0.30f},
    {0.00f, 0.10f, 0.10f, 0.20f},
    {1.40f, 0.10f, 1.50f, 0.20f}
};

// 4 vị trí có thể đặt vật cản ngẫu nhiên (R0..R3) ở góc phải và gần đích
static const Box2D_t RANDOM_OBS_BOXES[4] = {
    {1.45f, 0.90f, 1.55f, 1.00f}, // R0
    {1.60f, 0.65f, 1.70f, 0.75f}, // R1
    {1.25f, 0.28f, 1.35f, 0.38f}, // R2
    {1.55f, 0.03f, 1.65f, 0.13f}  // R3
};

// Tuyến đường chung từ XUẤT PHÁT (0.30, 0.05) đến điểm QUAN SÁT Q (1.25, 0.64)
static const Waypoint_t ROUTE_COMMON[] = {
    {0.30f, 0.05f},
    {0.13f, 0.39f},
    {0.61f, 0.94f},
    {0.95f, 0.80f},
    {1.10f, 0.75f},
    {1.25f, 0.64f}  // Điểm Q: Dừng quét vật cản ngẫu nhiên
};
#define LEN_COMMON  (sizeof(ROUTE_COMMON) / sizeof(Waypoint_t))

// 4 nhánh đường đi từ điểm Q về ĐÍCH (1.50, -0.05) ứng với 4 trường hợp R0..R3
static const Waypoint_t ROUTE_BRANCH[4][4] = {
    // Nhánh 0 (Khi vật ngẫu nhiên ở R0)
    {{1.25f, 0.64f}, {1.28f, 0.56f}, {1.32f, 0.02f}, {1.50f, -0.05f}},
    // Nhánh 1 (Khi vật ngẫu nhiên ở R1)
    {{1.25f, 0.64f}, {1.28f, 0.56f}, {1.32f, 0.02f}, {1.50f, -0.05f}},
    // Nhánh 2 (Khi vật ngẫu nhiên ở R2 - chặn khe hẹp 1.25..1.35 -> Đi vòng sang phải)
    {{1.25f, 0.64f}, {1.58f, 0.58f}, {1.58f, 0.02f}, {1.50f, -0.05f}},
    // Nhánh 3 (Khi vật ngẫu nhiên ở R3)
    {{1.25f, 0.64f}, {1.28f, 0.56f}, {1.32f, 0.02f}, {1.50f, -0.05f}}
};

// Góc lắp đặt 5 cảm biến ToF trên thân xe (rad): Giữa 0°, Trái +25°, Phải -25°, Hông trái +90°, Hông phải -90°
static const float TOF_MOUNT_ANGLE[NUM_TOF_SENSORS] = {
    0.0f, 0.43633f, -0.43633f, 1.570796f, -1.570796f
};
static const float TOF_OFFSET_R = 0.055f; // Khoảng cách từ tâm trục bánh tới mắt ToF (~5.5 cm)

/* ==================== 2. BIẾN TRẠNG THÁI TOÀN CỤC CỦA CORE ==================== */
static float ekf_x[3];          // [x (m), y (m), theta (rad)]
static float ekf_P[3][3];       // Ma trận hiệp phương sai 3x3

static MissionState_t state;
static uint32_t tick_ms;
static uint32_t state_timer_ms;

static float gyro_bias;
static float gyro_sum;
static uint32_t gyro_samples;

static int32_t prev_enc_L, prev_enc_R;
static float pi_integ_L, pi_integ_R;
static float target_v, target_w;
static float manual_v, manual_w;

static const Waypoint_t *active_route;
static uint8_t route_len;
static uint8_t wp_idx;          // Đang đi từ active_route[wp_idx-1] -> active_route[wp_idx]
static bool is_rotating_phase;  // true = Xoay tại chỗ (MO_ROTATE), false = Đi thẳng (MO_DRIVE)
static uint8_t chosen_branch;

/* ==================== 3. HÀM TIỆN ÍCH TOÁN HỌC & RAYCAST ==================== */
static float wrap_pi(float a) {
    while (a >  3.14159265f) a -= 6.28318530f;
    while (a < -3.14159265f) a += 6.28318530f;
    return a;
}

/* Giao điểm của tia (ox, oy) hướng (dx, dy) với 1 hộp chữ nhật Box2D */
static float Ray_Intersect_Box(float ox, float oy, float dx, float dy, Box2D_t b) {
    float tmin = 0.0f, tmax = 2.0f; // Giới hạn tầm đo ToF 2.0 m
    if (fabsf(dx) < 1e-6f) {
        if (ox < b.x_min || ox > b.x_max) return 2.0f;
    } else {
        float t1 = (b.x_min - ox) / dx;
        float t2 = (b.x_max - ox) / dx;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return 2.0f;
    }
    if (fabsf(dy) < 1e-6f) {
        if (oy < b.y_min || oy > b.y_max) return 2.0f;
    } else {
        float t1 = (b.y_min - oy) / dy;
        float t2 = (b.y_max - oy) / dy;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return 2.0f;
    }
    return (tmin > 0.0f) ? tmin : 2.0f;
}

/* Bắn tia Raycast từ pose (px, py, ptheta) cho cảm biến ToF thứ sensor_id */
static float Map_Raycast(float px, float py, float ptheta, uint8_t sensor_id, int8_t hypo_random_idx) {
    float ray_ang = ptheta + TOF_MOUNT_ANGLE[sensor_id];
    float ox = px + TOF_OFFSET_R * cosf(ray_ang);
    float oy = py + TOF_OFFSET_R * sinf(ray_ang);
    float dx = cosf(ray_ang);
    float dy = sinf(ray_ang);

    float min_d = 1.5f; // Tầm xa tối đa tin cậy

    // 1. Kiểm tra 17 vật cản tĩnh + Vùng cấm chữ U
    for (int i = 0; i <= NUM_STATIC_OBS; i++) {
        float d = Ray_Intersect_Box(ox, oy, dx, dy, MAP_OBSTACLES[i]);
        if (d < min_d) min_d = d;
    }
    // 2. Nếu đang thử giả thuyết vật ngẫu nhiên (0..3)
    if (hypo_random_idx >= 0 && hypo_random_idx < 4) {
        float d = Ray_Intersect_Box(ox, oy, dx, dy, RANDOM_OBS_BOXES[hypo_random_idx]);
        if (d < min_d) min_d = d;
    }
    return min_d;
}

/* ==================== 4. BỘ LỌC EKF BA TRẠNG THÁI (X, Y, THETA) ==================== */
static void EKF_Init(float x0, float y0, float theta0) {
    ekf_x[0] = x0;
    ekf_x[1] = y0;
    ekf_x[2] = theta0;
    memset(ekf_P, 0, sizeof(ekf_P));
    ekf_P[0][0] = 1e-4f; // Sai số ban đầu ~1 cm
    ekf_P[1][1] = 1e-4f;
    ekf_P[2][2] = 1e-3f;
}

static void EKF_Predict(float ds, float dtheta) {
    float th_mid = ekf_x[2] + 0.5f * dtheta;
    float c = cosf(th_mid);
    float s = sinf(th_mid);

    // Cập nhật trạng thái
    ekf_x[0] += ds * c;
    ekf_x[1] += ds * s;
    ekf_x[2]  = wrap_pi(ekf_x[2] + dtheta);

    // Jacobian F: F[0][2] = -ds*sin, F[1][2] = ds*cos
    float f02 = -ds * s;
    float f12 =  ds * c;

    // P = F * P * F^T + Q
    float p02 = ekf_P[0][2] + f02 * ekf_P[2][2];
    float p12 = ekf_P[1][2] + f12 * ekf_P[2][2];
    ekf_P[0][0] += 2.0f * f02 * ekf_P[0][2] + f02 * f02 * ekf_P[2][2] + 1e-5f;
    ekf_P[1][1] += 2.0f * f12 * ekf_P[1][2] + f12 * f12 * ekf_P[2][2] + 1e-5f;
    ekf_P[2][2] += 2e-5f;
    ekf_P[0][2] = ekf_P[2][0] = p02;
    ekf_P[1][2] = ekf_P[2][1] = p12;
}

static void EKF_Update_ToF(uint8_t sensor_id, float z_meas) {
    if (z_meas < 0.03f || z_meas > 0.80f) return; // Chỉ tin ToF trong tầm 3 cm .. 80 cm

    float h0 = Map_Raycast(ekf_x[0], ekf_x[1], ekf_x[2], sensor_id, -1);
    if (h0 >= 1.4f) return; // Tia nhìn ra khoảng không, bỏ qua

    // Tính Jacobian H = [dh/dx, dh/dy, dh/dtheta] bằng sai phân hữu hạn
    const float eps_xy = 0.005f; // 5 mm
    const float eps_th = 0.0087f; // 0.5 deg
    float H[3];
    H[0] = (Map_Raycast(ekf_x[0] + eps_xy, ekf_x[1], ekf_x[2], sensor_id, -1) - h0) / eps_xy;
    H[1] = (Map_Raycast(ekf_x[0], ekf_x[1] + eps_xy, ekf_x[2], sensor_id, -1) - h0) / eps_xy;
    H[2] = (Map_Raycast(ekf_x[0], ekf_x[1], ekf_x[2] + eps_th, sensor_id, -1) - h0) / eps_th;

    float nu = z_meas - h0; // Innovation
    float sigma_z = 0.01f + 0.03f * z_meas;
    float R = sigma_z * sigma_z;

    // Tính S = H * P * H^T + R
    float PHt[3];
    for (int i = 0; i < 3; i++) {
        PHt[i] = ekf_P[i][0] * H[0] + ekf_P[i][1] * H[1] + ekf_P[i][2] * H[2];
    }
    float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;

    // Cổng Mahalanobis: nu^2 / S > 9.0 -> Loại bỏ số đo nhiễu sượt cạnh
    if ((nu * nu) / S > 9.0f) return;

    // Kalman Gain K = PHt / S
    float K[3] = { PHt[0] / S, PHt[1] / S, PHt[2] / S };

    // Cập nhật trạng thái X và ma trận P
    ekf_x[0] += K[0] * nu;
    ekf_x[1] += K[1] * nu;
    ekf_x[2]  = wrap_pi(ekf_x[2] + K[2] * nu);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf_P[i][j] -= K[i] * PHt[j];
        }
    }
}

/* ==================== 5. ĐIỀU KHIỂN CHUYỂN ĐỘNG (ROTATE-THEN-DRIVE) ==================== */
static void Motion_Step_5ms(void) {
    if (wp_idx >= route_len) {
        target_v = 0.0f;
        target_w = 0.0f;
        return;
    }

    Waypoint_t a = active_route[wp_idx - 1];
    Waypoint_t b = active_route[wp_idx];
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float seg_len = sqrtf(dx * dx + dy * dy);
    float psi_seg = atan2f(dy, dx);

    if (is_rotating_phase) {
        /* Giai đoạn 1: Xoay tại chỗ hướng thẳng về Waypoint b */
        float psi_to_b = atan2f(b.y - ekf_x[1], b.x - ekf_x[0]);
        float e_th = wrap_pi(psi_to_b - ekf_x[2]);

        if (fabsf(e_th) < 0.025f) { // Sai số góc < ~1.5 độ -> Chuyển sang đi thẳng
            is_rotating_phase = false;
            target_w = 0.0f;
        } else {
            target_v = 0.0f;
            float w_cmd = 4.5f * e_th;
            if (w_cmd >  2.5f) w_cmd =  2.5f;
            if (w_cmd < -2.5f) w_cmd = -2.5f;
            target_w = w_cmd;
        }
    } else {
        /* Giai đoạn 2: Đi thẳng bám đoạn a -> b */
        float ux = dx / (seg_len + 1e-6f);
        float uy = dy / (seg_len + 1e-6f);
        float rx = ekf_x[0] - a.x;
        float ry = ekf_x[1] - a.y;

        float s_along = rx * ux + ry * uy;          // Quãng đường đã đi trên đoạn
        float e_y     = -rx * uy + ry * ux;         // Sai số lệch ngang (Cross-track error)
        float rem     = seg_len - s_along;          // Quãng đường còn lại tới b

        if (rem <= 0.005f) { // Còn <= 5 mm -> Chuyển sang đoạn kế tiếp
            wp_idx++;
            is_rotating_phase = true;
            target_v = 0.0f;
            target_w = 0.0f;
            return;
        }

        // Luật dẫn đường phi tuyến kéo xe về tâm đoạn thẳng
        float psi_des = psi_seg - atanf(6.0f * e_y);
        float e_th = wrap_pi(psi_des - ekf_x[2]);
        target_w = 5.0f * e_th;

        // Profile giảm tốc mượt khi gần tới điểm b (V_max = 0.22 m/s, A_dec = 0.35 m/s^2)
        float v_brake = sqrtf(2.0f * 0.35f * rem);
        float v_cmd = (v_brake < 0.22f) ? v_brake : 0.22f;
        if (v_cmd < 0.04f) v_cmd = 0.04f;
        target_v = v_cmd * fmaxf(0.0f, cosf(e_th));
    }
}

/* ==================== 6. KHỞI TẠO & HÀM BƯỚC 1 MS ==================== */
void RobotCore_Init(void) {
    state = STATE_IDLE;
    tick_ms = 0;
    gyro_bias = 0.0f;
    target_v = 0.0f;
    target_w = 0.0f;
    pi_integ_L = 0.0f;
    pi_integ_R = 0.0f;
    EKF_Init(ROUTE_COMMON[0].x, ROUTE_COMMON[0].y, 1.570796f); // Đặt hướng ban đầu 90° (nhìn lên)
}

void RobotCore_SetManualCmd(float v_m_s, float w_rad_s) {
    manual_v = v_m_s;
    manual_w = w_rad_s;
}

void RobotCore_Step_1ms(const SensorFrame_t *in, ActuatorCmd_t *out) {
    tick_ms++;

    // 1. Tính vi phân xung Encoder mỗi 1 ms
    int32_t dL = in->enc_left_ticks  - prev_enc_L;
    int32_t dR = in->enc_right_ticks - prev_enc_R;
    prev_enc_L = in->enc_left_ticks;
    prev_enc_R = in->enc_right_ticks;

    float vL_meas = (dL * METERS_PER_TICK) * 1000.0f;
    float vR_meas = (dR * METERS_PER_TICK) * 1000.0f;

    // 2. Chế độ Lái tay không dây (Lượt 1)
    if (!in->is_auto_mode) {
        target_v = manual_v;
        target_w = manual_w;
        out->led_mask = 0x01;
    } else {
        // 3. Chế độ Tự hành (Lượt 2) - Phát hiện nhấc xe khi cả 4 IR đều vọt cao (mất phản xạ sàn)
        if (state != STATE_IDLE && state != STATE_CALIB && state != STATE_LIFTED) {
            if (in->ir_adc[0] > 3200 && in->ir_adc[1] > 3200 && in->ir_adc[2] > 3200 && in->ir_adc[3] > 3200) {
                state = STATE_LIFTED;
                target_v = 0.0f;
                target_w = 0.0f;
            }
        }

        // Cập nhật EKF ngay khi có mẫu ToF mới
        if (state == STATE_RUN_COMMON || state == STATE_RUN_BRANCH) {
            for (uint8_t i = 0; i < NUM_TOF_SENSORS; i++) {
                if (in->tof_dist_m[i] > 0.0f) {
                    EKF_Update_ToF(i, in->tof_dist_m[i]);
                }
            }
        }

        // Nhịp 200 Hz (Mỗi 5 ms): Chạy EKF Predict + Máy trạng thái + Motion
        if (tick_ms % 5 == 0) {
            float dt = 0.005f;
            float ds = 0.5f * (vL_meas + vR_meas) * dt;
            float dtheta = (in->gyro_z_rad_s - gyro_bias) * dt;

            if (state == STATE_RUN_COMMON || state == STATE_RUN_BRANCH) {
                EKF_Predict(ds, dtheta);
            }

            switch (state) {
                case STATE_IDLE:
                    target_v = 0.0f; target_w = 0.0f;
                    out->led_mask = 0x01;
                    if (in->btn_start_pressed) {
                        state = STATE_CALIB;
                        state_timer_ms = tick_ms;
                        gyro_sum = 0.0f;
                        gyro_samples = 0;
                    }
                    break;

                case STATE_CALIB:
                    target_v = 0.0f; target_w = 0.0f;
                    out->led_mask = 0x03;
                    gyro_sum += in->gyro_z_rad_s;
                    gyro_samples++;
                    if (tick_ms - state_timer_ms >= 1500) { // Đo bias 1.5 giây
                        gyro_bias = gyro_sum / (float)(gyro_samples ? gyro_samples : 1);
                        EKF_Init(ROUTE_COMMON[0].x, ROUTE_COMMON[0].y, 1.570796f);
                        active_route = ROUTE_COMMON;
                        route_len = LEN_COMMON;
                        wp_idx = 1;
                        is_rotating_phase = true;
                        state = STATE_RUN_COMMON;
                    }
                    break;

                case STATE_RUN_COMMON:
                    out->led_mask = 0x04;
                    Motion_Step_5ms();
                    if (wp_idx >= route_len) {
                        state = STATE_OBSERVE;
                        state_timer_ms = tick_ms;
                    }
                    break;

                case STATE_OBSERVE:
                    /* Dừng tại Q 300ms để chọn nhánh R0..R3 dựa trên ToF */
                    target_v = 0.0f; target_w = 0.0f;
                    out->led_mask = 0x0F;
                    if (tick_ms - state_timer_ms >= 300) {
                        // Kiểm tra nếu ToF trước thấy vật cản ở R2 (chặn đường thẳng xuống đích)
                        if (in->tof_dist_m[0] > 0.05f && in->tof_dist_m[0] < 0.35f) {
                            chosen_branch = 2; // Đi nhánh vòng R2
                        } else {
                            chosen_branch = 0; // Đi nhánh thẳng tiêu chuẩn
                        }
                        active_route = ROUTE_BRANCH[chosen_branch];
                        route_len = 4;
                        wp_idx = 1;
                        is_rotating_phase = true;
                        state = STATE_RUN_BRANCH;
                    }
                    break;

                case STATE_RUN_BRANCH:
                    out->led_mask = 0x08;
                    Motion_Step_5ms();
                    if (wp_idx >= route_len) {
                        state = STATE_FINISH;
                    }
                    break;

                case STATE_LIFTED:
                    /* Xe bị nhấc lên -> Tắt động cơ, chờ đặt xuống vạch và nhấn START */
                    target_v = 0.0f; target_w = 0.0f;
                    out->led_mask = (tick_ms / 100) % 2 ? 0x0F : 0x00;
                    if (in->btn_start_pressed) {
                        // Dùng DIP Switch (0..3) để chọn nhanh Waypoint đặt lại xe
                        uint8_t resume_wp = (in->dip_val < LEN_COMMON - 1) ? in->dip_val : 0;
                        EKF_Init(ROUTE_COMMON[resume_wp].x, ROUTE_COMMON[resume_wp].y, 1.570796f);
                        active_route = ROUTE_COMMON;
                        route_len = LEN_COMMON;
                        wp_idx = resume_wp + 1;
                        is_rotating_phase = true;
                        state = STATE_RUN_COMMON;
                    }
                    break;

                case STATE_FINISH:
                    target_v = 0.0f; target_w = 0.0f;
                    out->led_mask = 0x0F;
                    break;
            }
        }
    }

    // 4. Tính vận tốc đặt 2 bánh và chạy PI 1 kHz
    float vL_ref = target_v - 0.5f * WHEEL_BASE_M * target_w;
    float vR_ref = target_v + 0.5f * WHEEL_BASE_M * target_w;

    if (fabsf(vL_ref) < 0.001f && fabsf(vR_ref) < 0.001f) {
        pi_integ_L = 0.0f;
        pi_integ_R = 0.0f;
        out->pwm_left = 0;
        out->pwm_right = 0;
        return;
    }

    float err_L = vL_ref - vL_meas;
    float err_R = vR_ref - vR_meas;
    pi_integ_L += err_L * 0.001f;
    pi_integ_R += err_R * 0.001f;

    // Chống bão hòa tích phân (Anti-windup)
    if (pi_integ_L >  0.3f) pi_integ_L =  0.3f;
    if (pi_integ_L < -0.3f) pi_integ_L = -0.3f;
    if (pi_integ_R >  0.3f) pi_integ_R =  0.3f;
    if (pi_integ_R < -0.3f) pi_integ_R = -0.3f;

    // Feedforward + PI (Đổi sang thang PWM -2940 .. +2940)
    float uL = 6500.0f * vL_ref + 4500.0f * err_L + 12000.0f * pi_integ_L;
    float uR = 6500.0f * vR_ref + 4500.0f * err_R + 12000.0f * pi_integ_R;

    if (uL >  2940.0f) uL =  2940.0f;
    if (uL < -2940.0f) uL = -2940.0f;
    if (uR >  2940.0f) uR =  2940.0f;
    if (uR < -2940.0f) uR = -2940.0f;

    out->pwm_left  = (int16_t)uL;
    out->pwm_right = (int16_t)uR;
}
```

---

### 3.3. File Ghép Nối Phần Cứng STM32 (`Core/Src/main_integration.c`)

Đoạn mã dưới đây kết nối lớp `Core` với các ngoại vi của STM32 (Ngắt `TIM5`, đọc `MPU6050` trong vòng lặp chính và nhận lệnh lái tay qua Bluetooth `USART6` cho Lượt 1):

```c
#include "main.h"
#include "robot_core.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart6;

static volatile SensorFrame_t g_sensor_frame;
static ActuatorCmd_t g_actuator_cmd;
static uint8_t bt_rx_byte;

/* Nhận lệnh điều khiển không dây qua Bluetooth HC-06 (Lượt 1)
 * Gửi ký tự: 'W' (Tiến), 'S' (Lùi), 'A' (Quay trái), 'D' (Quay phải), 'X' (Dừng)
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        switch (bt_rx_byte) {
            case 'W': case 'w': RobotCore_SetManualCmd( 0.20f,  0.0f); break;
            case 'S': case 's': RobotCore_SetManualCmd(-0.15f,  0.0f); break;
            case 'A': case 'a': RobotCore_SetManualCmd( 0.0f,   2.5f); break;
            case 'D': case 'd': RobotCore_SetManualCmd( 0.0f,  -2.5f); break;
            default:            RobotCore_SetManualCmd( 0.0f,   0.0f); break;
        }
        HAL_UART_Receive_IT(&huart6, &bt_rx_byte, 1);
    }
}

/* Ngắt Timer 5 chạy cố định mỗi 1 ms (1000 Hz) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5) {
        // 1. Đọc xung phần cứng từ TIM2 (Trái 32-bit) và TIM1 (Phải 16-bit mở rộng 32-bit)
        static uint16_t prev_tim1 = 0;
        static int32_t  enc_right_32 = 0;
        uint16_t cur_tim1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
        int16_t  diff_tim1 = (int16_t)(cur_tim1 - prev_tim1);
        prev_tim1 = cur_tim1;
        enc_right_32 += diff_tim1;

        g_sensor_frame.enc_left_ticks  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        g_sensor_frame.enc_right_ticks = enc_right_32;

        // 2. Đọc nút bấm & công tắc
        g_sensor_frame.btn_start_pressed = (HAL_GPIO_ReadPin(INPUT_PORT, BTN_START_PIN) == GPIO_PIN_RESET);
        g_sensor_frame.is_auto_mode      = (HAL_GPIO_ReadPin(INPUT_PORT, SW_AUTO_MAN_PIN) == GPIO_PIN_RESET);
        uint8_t b1 = (HAL_GPIO_ReadPin(INPUT_PORT, DIP_BIT1_PIN) == GPIO_PIN_RESET) ? 1 : 0;
        uint8_t b2 = (HAL_GPIO_ReadPin(INPUT_PORT, DIP_BIT2_PIN) == GPIO_PIN_RESET) ? 1 : 0;
        g_sensor_frame.dip_val = (b2 << 1) | b1;

        // 3. Chạy lõi thuật toán 1 ms
        SensorFrame_t frame_copy = g_sensor_frame;
        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            g_sensor_frame.tof_dist_m[i] = -1.0f; // Xóa cờ sau khi đã copy vào Core
        }
        RobotCore_Step_1ms(&frame_copy, &g_actuator_cmd);

        // 4. Xuất PWM ra DRV8833 và cập nhật 4 LED
        extern void Motor_Set_PWM(int16_t left_speed, int16_t right_speed);
        Motor_Set_PWM(g_actuator_cmd.pwm_left, g_actuator_cmd.pwm_right);

        HAL_GPIO_WritePin(LED_PORT, LED0_PIN, (g_actuator_cmd.led_mask & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_PORT, LED1_PIN, (g_actuator_cmd.led_mask & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, (g_actuator_cmd.led_mask & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_PORT, LED3_PIN, (g_actuator_cmd.led_mask & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
```

---

## 4. Quy Trình Vận Hành Thực Tế Khi Đi Thi

1. **Lượt 1 (Kiểm tra điều khiển không dây - Tối đa 3 phút):**
   * Gạt công tắc `SW5` (`PB13`) sang chế độ **Manual**.
   * Kết nối Laptop/Điện thoại với Bluetooth `HC-06`, dùng phím `W, A, S, D` lái xe về đích nhanh nhất có thể để tiết kiệm thời gian cộng dồn sang Lượt 2.
2. **Lượt 2 (Chạy tự động tính điểm + Vật cản ngẫu nhiên):**
   * Gạt công tắc `SW5` sang chế độ **Auto**.
   * Đặt tâm trục bánh xe đúng tại tọa độ xuất phát `(0.30m, 0.05m)`, mũi xe hướng thẳng góc $90^\circ$ trước khi khóa xe và bốc thăm.
   * Khi trọng tài hô bắt đầu, nhấn nút **`START` (`PB12`)**:
     * **1.5 giây đầu (`STATE_CALIB`):** Giữ tay khỏi xe để xe đứng yên tuyệt đối lấy mẫu `gyro_bias` (2 LED sáng).
     * **Chạy đoạn chung (`STATE_RUN_COMMON`):** Xe tự động xoay và đi qua các khe rộng $\ge 18\text{ cm}$ tới điểm `Q`.
     * **Tại điểm `Q` (`STATE_OBSERVE`):** Xe dừng $0.3\text{ s}$, đọc ToF phía trước để phát hiện vị trí vật cản ngẫu nhiên và tự rẽ vào nhánh trống về thẳng vạch Đích.