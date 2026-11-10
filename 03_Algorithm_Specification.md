# ĐẶC TẢ CHI TIẾT THUẬT TOÁN HỆ THỐNG XE DÒ MÊ CUNG (`xe_dua`)

* **Dự án:** Xe tự hành vượt vật cản dựa trên bản đồ biết trước
* **Vi điều khiển mục tiêu:** STM32F401RCT6 (Cortex-M4F, đơn vị tính toán dấu phẩy động FPU 32-bit)
* **Triết lý thiết kế:** Biến bài toán phản xạ cục bộ thành bài toán **Định vị xác suất (EKF) + Bám quỹ đạo tất định (Deterministic Path Tracking)**.

---

## 1. Tổng Quan Kiến Trúc Thuật Toán & Phân Tầng Tần Số

Toàn bộ thuật toán được chia thành **2 giai đoạn (Offline trên PC và Online trên STM32)** và chạy ở **3 miền tần số** khác nhau để tối ưu hóa tải CPU:

| Tầng xử lý | Nơi chạy | Tần số | Thuật toán đảm nhiệm | Đầu vào $\rightarrow$ Đầu ra |
| :--- | :---: | :---: | :--- | :--- |
| **Lập kế hoạch (Planning)** | PC (Python) | Offline | Nở vật cản (Minkowski Sum), Tìm đường $A^*$ 8 hướng có trọng số an toàn, Rút gọn Waypoint | Bản đồ 17 vật tĩnh + 4 vị trí ngẫu nhiên $\rightarrow$ Tập tuyến đường `ROUTE_COMMON`, `ROUTE_R0..R3` |
| **Cảm biến chậm (I2C Polling)** | STM32 (`main`) | $30\text{ Hz} - 500\text{ Hz}$ | Đọc lệch tầng (Staggered) 5 cảm biến ToF `VL53L0X` và Gyro Z `MPU6050` | Tín hiệu vật lý $\rightarrow$ Khoảng cách $z_i\text{ [m]}$, Vận tốc góc $\omega_z\text{ [rad/s]}$ |
| **Định vị & Dẫn đường (Navigation)** | STM32 (`TIM5`) | $200\text{ Hz}$ ($5\text{ ms}$) | Odometry bậc 2, Bộ lọc Kalman mở rộng (EKF), Raycast, Bám đường phi tuyến, Máy trạng thái + Huber | Xung Encoder, $\omega_z$, $z_i$ $\rightarrow$ Pose ước lượng $\hat{x}=[x,y,\theta]^T$ & Lệnh $(v_{\text{cmd}}, \omega_{\text{cmd}})$ |
| **Điều khiển chấp hành (Control)** | STM32 (`TIM5`) | $1000\text{ Hz}$ ($1\text{ ms}$) | Động học ngược vi sai, Bộ điều khiển PI + Feedforward + Anti-windup | $(v_{\text{cmd}}, \omega_{\text{cmd}})$ $\rightarrow$ Xung PWM $20\text{ kHz}$ cho cầu H `DRV8833` |

---

## 2. Hệ Trục Tọa Độ và Mô Hình Động Học Xe Vi Sai

### 2.1. Quy ước hệ trục tọa độ
1. **Hệ tọa độ Bản đồ toàn cục (Global Frame $\{W\}$):**
   * Gốc tọa độ $O_W(0, 0)$ đặt tại góc dưới cùng bên trái của sân thi đấu ($1.80\text{ m} \times 1.20\text{ m}$).
   * Trục $X_W$ hướng sang phải dọc theo chiều dài $1.80\text{ m}$.
   * Trục $Y_W$ hướng lên trên dọc theo chiều rộng $1.20\text{ m}$.
   * Góc hướng xe $\theta \in (-\pi, \pi]$ tính từ trục $X_W$ dương, ngược chiều kim đồng hồ (tại vạch xuất phát, xe nhìn thẳng lên trên ứng với $\theta_0 = +\frac{\pi}{2}\text{ rad}$).
2. **Hệ tọa độ Thân xe (Body Frame $\{B\}$):**
   * Gốc $O_B$ đặt tại **trung điểm trục nối 2 bánh xe chủ động**.
   * Trục $X_B$ hướng thẳng về phía mũi xe; trục $Y_B$ hướng sang bên trái xe.

### 2.2. Mô hình động học thuận và ngược (Differential Drive Kinematics)
Gọi $r_w$ là bán kính bánh xe, $b$ là khoảng cách giữa hai tâm bánh xe (wheelbase), và $N_{\text{PPR}}$ là số xung Encoder đếm được ở chế độ $4\times$ trên một vòng quay bánh xe.

* **Quãng đường dịch chuyển của từng bánh xe trong chu kỳ lấy mẫu $\Delta t$:**
  $$\Delta s_L = \frac{2\pi r_w}{N_{\text{PPR}}} \Delta N_L, \quad \Delta s_R = \frac{2\pi r_w}{N_{\text{PPR}}} \Delta N_R$$
* **Vận tốc dài và vận tốc góc của tâm xe:**
  $$v = \frac{v_R + v_L}{2}, \quad \omega = \frac{v_R - v_L}{b}$$
* **Động học ngược (từ lệnh điều khiển $(v_{\text{cmd}}, \omega_{\text{cmd}})$ ra vận tốc đặt 2 bánh):**
  $$v_{L,\text{ref}} = v_{\text{cmd}} - \frac{b}{2}\omega_{\text{cmd}}, \quad v_{R,\text{ref}} = v_{\text{cmd}} + \frac{b}{2}\omega_{\text{cmd}}$$

---

## 3. Thuật Toán Lập Đường Đi Offline ($A^*$ Trên Không Gian Cấu Hình)

Vì 17 khối vật cản tĩnh ($10 \times 10\text{ cm}$) và 4 vị trí bốc thăm vật cản ngẫu nhiên đã biết trước tọa độ, toàn bộ việc tìm đường nặng về tính toán được thực hiện offline bằng Python để xuất ra mảng hằng số `const Waypoint_t` nạp vào Flash của STM32.

### 3.1. Không gian cấu hình (Configuration Space - C-Space)
Kích thước bao của xe: rộng $83\text{ mm}$, tâm trục bánh cách mũi xe $60\text{ mm}$ và cách đuôi xe $39\text{ mm}$. Bán kính đường tròn ngoại tiếp quét quanh tâm trục bánh xe khi xoay tại chỗ là:
$$R_{\text{robot}} = \sqrt{60^2 + \left(\frac{83}{2}\right)^2} \approx 60\text{ mm} = 6.0\text{ cm}$$

Để coi xe như một chất điểm $(x, y)$ duy nhất và đảm bảo xe có thể xoay tại bất kỳ Waypoint nào mà không quẹt góc vật cản:
* **Nở vật cản (Obstacle Inflation):** Mọi khối vật cản $10 \times 10\text{ cm}$ được nở rộng ra mọi phía một khoảng:
  $$d_{\text{inflate}} = R_{\text{robot}} + d_{\text{margin}} = 6.0\text{ cm} + 1.5\text{ cm} = 7.5\text{ cm}$$
* **Co biên làn đường:** Băng dính đen bao quanh sân và vùng cấm chữ U được đẩy vào trong một khoảng:
  $$d_{\text{border}} = \frac{W_{\text{robot}}}{2} + d_{\text{margin\_line}} = 4.15\text{ cm} + 2.0\text{ cm} = 6.15\text{ cm}$$

### 3.2. Thuật toán tìm đường $A^*$ đa mục tiêu trên lưới $0.5\text{ cm}$
Bản đồ $180 \times 120\text{ cm}$ được rời rạc hóa thành lưới $360 \times 240$ ô vuông kích thước $0.5\text{ cm}$. Hàm đánh giá tại mỗi nút $n$:
$$f(n) = g(n) + h(n)$$
Trong đó $h(n)$ là khoảng cách Octile (8 hướng) tới đích, còn chi phí tích lũy $g(n)$ được thiết kế đặc biệt để **ưu tiên khe rộng và giàu mốc định vị ToF**:
$$g(n_{\text{next}}) = g(n) + \Delta d(n, n_{\text{next}}) + w_{\text{clear}} \cdot C_{\text{clear}}(n_{\text{next}}) + w_{\text{turn}} \cdot C_{\text{turn}}(n, n_{\text{next}}) - w_{\text{lm}} \cdot B_{\text{landmark}}(n_{\text{next}})$$

* **$C_{\text{clear}}(n)$ (Phạt khoảng hở hẹp):** Sử dụng biến đổi khoảng cách Euclidean (EDT) tính khoảng cách gần nhất $d_{\text{obs}}(n)$ từ ô $n$ tới vật cản đã nở:
  $$C_{\text{clear}}(n) = \max\left(0, \frac{1}{d_{\text{obs}}(n)} - \frac{1}{d_{\text{safe}}}\right)^2$$
  Hàm này ép đường đi luôn nằm chính giữa khe hở giữa 2 vật cản, tự động tránh khe hẹp $15\text{ cm}$ và chọn khe $\ge 18\text{ cm}$.
* **$C_{\text{turn}}$ (Phạt đổi hướng):** Phạt mỗi khi hướng di chuyển thay đổi để giảm số lượng Waypoint.
* **$B_{\text{landmark}}$ (Thưởng độ phủ mốc ToF):** Thưởng nhẹ cho các ô mà từ đó các tia cảm biến ToF nhìn thấy mặt phẳng vật cản ở cự ly $10\text{--}40\text{ cm}$, giúp EKF luôn có số đo cập nhật khi xe chạy.
* **Hậu xử lý (Line-of-Sight Pruning):** Đường gấp khúc trên lưới được rút gọn bằng thuật toán Ramer-Douglas-Peucker kết hợp kiểm tra va chạm hình chữ nhật thật ($83 \times 99\text{ mm}$) quét dọc từng đoạn thẳng, thu được $6\text{--}9$ Waypoint cuối cùng.

---

## 4. Thuật Toán Định Vị Hợp Nhất Cảm Biến: EKF 3 Trạng Thái

Đây là "trái tim" của phần mềm chạy trên STM32 ở tần số $200\text{ Hz}$ ($5\text{ ms}$). Nếu chỉ dùng Encoder + Gyro (Dead-reckoning), sau quãng đường $\sim 2.8\text{ m}$ xe sẽ trôi $3\text{--}6\text{ cm}$ và đâm vào vật cản ở cuối sân. Bộ lọc **Extended Kalman Filter (EKF)** kết hợp Odometry với bản đồ 17 vật cản tĩnh thông qua 5 cảm biến laser ToF để giữ sai số vị trí luôn $< \pm 1.5\text{ cm}$.

### 4.1. Vector trạng thái và Ma trận hiệp phương sai
Vector trạng thái tại bước thời gian $k$:
$$x_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}, \quad P_k = \begin{bmatrix} \sigma_{xx}^2 & \sigma_{xy} & \sigma_{x\theta} \\ \sigma_{yx} & \sigma_{yy}^2 & \sigma_{y\theta} \\ \sigma_{\theta x} & \sigma_{\theta y} & \sigma_{\theta\theta}^2 \end{bmatrix} \in \mathbb{R}^{3\times 3}$$

### 4.2. Bước Dự đoán (Prediction Step – Chạy mỗi $5\text{ ms}$)
Thay vì lấy góc quay từ hiệu 2 bánh xe Encoder (rất dễ sai lệch khi bánh bị trượt), hệ thống **tách bạch nguồn đo**:
* Quãng đường tiến $\Delta s_k$ lấy từ trung bình 2 bánh Encoder:
  $$\Delta s_k = \frac{\Delta s_{L,k} + \Delta s_{R,k}}{2}$$
* Góc quay $\Delta \theta_k$ lấy từ tích phân vận tốc góc Gyro Z của `MPU6050` (đã trừ độ trôi `bias` và nhân hệ số `scale`):
  $$\Delta \theta_k = \left(\omega_{\text{gyro}, k} - b_{\text{gyro}}\right) \cdot S_{\text{gyro}} \cdot \Delta t$$

1. **Phương trình dự đoán trạng thái (Tích phân Runge-Kutta bậc 2 / Midpoint):**
   $$\theta_{\text{mid}} = \theta_{k-1} + \frac{1}{2}\Delta \theta_k$$
   $$\hat{x}_{k|k-1} = f(\hat{x}_{k-1}, \Delta s_k, \Delta \theta_k) = \begin{bmatrix} \hat{x}_{k-1} + \Delta s_k \cos(\theta_{\text{mid}}) \\ \hat{y}_{k-1} + \Delta s_k \sin(\theta_{\text{mid}}) \\ \text{wrap}_{\pi}(\hat{\theta}_{k-1} + \Delta \theta_k) \end{bmatrix}$$

2. **Ma trận Jacobian chuyển trạng thái $F_k = \left.\frac{\partial f}{\partial x}\right|_{\hat{x}_{k-1}}$:**
   $$F_k = \begin{bmatrix} 1 & 0 & -\Delta s_k \sin(\theta_{\text{mid}}) \\ 0 & 1 & \Delta s_k \cos(\theta_{\text{mid}}) \\ 0 & 0 & 1 \end{bmatrix}$$

3. **Ma trận nhiễu quá trình $Q_k$ (Tỷ lệ thuận với quãng đường và góc di chuyển):**
   $$Q_k = \begin{bmatrix} (\alpha_s |\Delta s_k|)^2 + q_{xy,0} & 0 & 0 \\ 0 & (\alpha_s |\Delta s_k|)^2 + q_{xy,0} & 0 \\ 0 & 0 & (\alpha_\theta |\Delta \theta_k|)^2 + (\sigma_{\text{gyro}} \Delta t)^2 \end{bmatrix}$$

4. **Cập nhật ma trận hiệp phương sai tiên nghiệm:**
   $$P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k$$

### 4.3. Mô hình Đo lường ToF bằng Raycast 3 Tia (Multi-Ray Beam Model)
Khi cảm biến ToF thứ $i \in \{0..4\}$ trả về số đo khoảng cách hợp lệ $z_i \in [0.03\text{ m}, 0.80\text{ m}]$:
* Gọi $(l_{x,i}, l_{y,i}, \alpha_i)$ là tọa độ lắp đặt và góc hướng của cảm biến ToF thứ $i$ trong hệ quy chiếu thân xe $\{B\}$ (với $\alpha \in \{0^\circ, +25^\circ, -25^\circ, +90^\circ, -90^\circ\}$).
* Tọa độ mắt đọc cảm biến trong hệ toàn cục $\{W\}$:
  $$\begin{bmatrix} x_{s,i} \\ y_{s,i} \end{bmatrix} = \begin{bmatrix} \hat{x} \\ \hat{y} \end{bmatrix} + \begin{bmatrix} \cos\hat{\theta} & -\sin\hat{\theta} \\ \sin\hat{\theta} & \cos\hat{\theta} \end{bmatrix} \begin{bmatrix} l_{x,i} \\ l_{y,i} \end{bmatrix}, \quad \phi_i = \hat{\theta} + \alpha_i$$

* **Kỹ thuật Raycast 3 tia mô phỏng chùm tia hình nón (FoV $25^\circ$):**
  Vì tia laser của `VL53L0X` không phải là một đường thẳng lý tưởng mà có độ mở chùm tia, hàm dự đoán số đo $h_i(\hat{x})$ bắn **3 tia ảo** ở các góc $\phi_i$, $\phi_i - 10^\circ$, $\phi_i + 10^\circ$ vào danh sách các cạnh hộp chữ nhật AABB của 17 khối vật cản tĩnh (sử dụng thuật toán giao cắt đoạn thẳng Slab Method):
  $$h_i(\hat{x}) = \min_{\delta \in \{-10^\circ, 0^\circ, +10^\circ\}} \text{Raycast}_{\text{AABB}}\left(x_{s,i}, y_{s,i}, \phi_i + \delta\right)$$

### 4.4. Tính Jacobian số, Cổng Mahalanobis và Cập nhật EKF
1. **Ma trận Jacobian đo lường $H_i \in \mathbb{R}^{1\times 3}$ bằng sai phân hữu hạn:**
   Do hàm Raycast chứa phép lấy $\min$ trên các cạnh hộp nên không khả vi giải tích, ta tính đạo hàm số với bước nhiễu nhỏ $\epsilon_{xy} = 5\text{ mm}$ và $\epsilon_\theta = 0.5^\circ \approx 0.00873\text{ rad}$:
   $$H_i = \begin{bmatrix} \dfrac{h_i(\hat{x} + \epsilon_{xy}e_1) - h_i(\hat{x})}{\epsilon_{xy}} & \dfrac{h_i(\hat{x} + \epsilon_{xy}e_2) - h_i(\hat{x})}{\epsilon_{xy}} & \dfrac{h_i(\hat{x} + \epsilon_\theta e_3) - h_i(\hat{x})}{\epsilon_\theta} \end{bmatrix}$$

2. **Phương sai nhiễu đo phụ thuộc khoảng cách:**
   $$R_i = (\sigma_{0} + \beta z_i)^2 = (0.01 + 0.03 z_i)^2$$

3. **Sai số đổi mới (Innovation) và Cổng loại nhiễu Mahalanobis:**
   $$\nu_i = z_i - h_i(\hat{x}_{k|k-1})$$
   $$S_i = H_i P_{k|k-1} H_i^T + R_i$$
   Kiểm định khoảng cách bình phương Mahalanobis:
   $$d_M^2 = \frac{\nu_i^2}{S_i} \le \gamma_{\text{gate}} = 9.0 \quad (\text{tương ứng ngưỡng } 3\sigma)$$
   * **Ý nghĩa vật lý:** Nếu tia laser bắn sượt qua mép khối gỗ, bị phản xạ đa đường dưới sàn hoặc bắn trúng một vật cản chưa biết, giá trị $|\nu_i|$ sẽ rất lớn làm $d_M^2 > 9$. Bộ lọc lập tức **bỏ qua số đo này**, bảo vệ EKF không bị kéo lệch.

4. **Công thức cập nhật trạng thái (Khi $d_M^2 \le 9.0$):**
   $$K_i = \frac{P_{k|k-1} H_i^T}{S_i} \in \mathbb{R}^{3\times 1}$$
   $$\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_i \nu_i$$
   $$P_{k|k} = (I_3 - K_i H_i) P_{k|k-1}$$

### 4.5. Cập nhật EKF bằng cảm biến hồng ngoại IR khi chạm mép băng đen
Khi một trong 4 mắt đọc IR dưới gầm xe $(j \in \{0..3\})$ phát hiện mức phản xạ vượt ngưỡng băng dính đen:
* Biết tọa độ mắt IR thứ $j$ trên thân xe, suy ra vị trí điểm chạm $(x_{\text{ir}}, y_{\text{ir}})$ trên bản đồ.
* Nếu điểm này nằm gần biên sân ($x \approx 0, x \approx 1.8, y \approx 0, y \approx 1.2$) hoặc biên vùng cấm chữ U, nó cung cấp một phép đo trực tiếp 1 chiều (theo phương vuông góc với vạch đen) với độ chính xác cực cao ($\sigma_{\text{ir}} \approx 5\text{ mm}$), giúp triệt tiêu hoàn toàn sai số vuông góc với biên đường đua.

---

## 5. Thuật Toán Điều Khiển Chuyển Động & Bám Quỹ Đạo

Để tránh hiện tượng xe vừa cua gấp vừa đi thẳng làm quẹt đuôi vào vật cản trong khe hẹp $18\text{ cm}$, thuật toán điều khiển chuyển động (`Motion`) tách việc di chuyển qua danh sách Waypoint $\{W_0, W_1, \dots, W_M\}$ thành **2 pha rõ rệt** trên từng đoạn thẳng $W_{m-1}(x_a, y_a) \rightarrow W_m(x_b, y_b)$:

### 5.1. Pha 1: Xoay tại chỗ (`MO_ROTATE`)
* Tính góc hướng mục tiêu từ vị trí hiện tại tới điểm đích của đoạn $W_m$:
  $$\psi_{\text{target}} = \text{atan2}(y_b - \hat{y}, x_b - \hat{x})$$
* Sai số góc hướng:
  $$e_\theta = \text{wrap}_\pi(\psi_{\text{target}} - \hat{\theta})$$
* Đặt vận tốc dài $v_{\text{cmd}} = 0$, vận tốc góc tuân theo profile hình thang giới hạn gia tốc góc:
  $$\omega_{\text{cmd}} = \text{sat}_{\omega_{\max}}\left(K_{\text{rot}} \cdot e_\theta\right)$$
* **Điều kiện chuyển pha:** Khi $|e_\theta| < 1.0^\circ$ ($0.0175\text{ rad}$) và giữ ổn định liên tiếp trong $50\text{ ms}$ (10 chu kỳ), chuyển sang pha đi thẳng `MO_DRIVE`.

### 5.2. Pha 2: Đi thẳng bám đoạn và Giảm tốc căn đích (`MO_DRIVE`)
Gọi vector đơn vị chỉ phương của đoạn thẳng $W_{m-1} \rightarrow W_m$ là $\mathbf{u} = [u_x, u_y]^T = \frac{1}{L}[x_b - x_a, y_b - y_a]^T$, góc đoạn thẳng là $\psi_{\text{seg}} = \text{atan2}(u_y, u_x)$.

1. **Chiếu vị trí EKF của xe lên đoạn thẳng:**
   * Quãng đường đã đi dọc theo đoạn:
     $$s = (\hat{x} - x_a)u_x + (\hat{y} - y_a)u_y$$
   * Quãng đường còn lại tới Waypoint $W_m$:
     $$\text{rem} = L - s$$
   * Sai số lệch ngang (Cross-track error $e_y$, dương khi xe lệch sang trái đoạn thẳng):
     $$e_y = -(\hat{x} - x_a)u_y + (\hat{y} - y_a)u_x$$

2. **Luật điều khiển hướng nhìn phi tuyến (Nonlinear Line-of-Sight Guidance):**
   $$\psi_{\text{des}} = \psi_{\text{seg}} - \arctan(K_y \cdot e_y)$$
   $$\omega_{\text{cmd}} = K_h \cdot \text{wrap}_\pi(\psi_{\text{des}} - \hat{\theta})$$
   * **Ưu điểm:** Khi $e_y$ lớn, hàm $\arctan$ giới hạn góc lao vào đường chuẩn tối đa không quá $45^\circ\text{--}60^\circ$, tránh vọt lố (overshoot); khi $e_y \to 0$, $\arctan(K_y e_y) \approx K_y e_y$ hoạt động như bộ điều khiển PD mượt mà.

3. **Profile vận tốc dài giảm tốc theo căn bậc hai khoảng cách:**
   $$v_{\text{cmd}} = \min\left(V_{\max}, \sqrt{2 A_{\text{dec}} \cdot \max(0, \text{rem})}\right) \cdot \max(0, \cos e_\theta)$$
   * Khi còn cách điểm dừng $W_m$ một khoảng ngắn, vận tốc tự động giảm tuyến tính theo thời gian với gia tốc hãm $A_{\text{dec}}$, giúp xe dừng chính xác tại đỉnh khúc cua với sai số $\text{rem} \le 3\text{ mm}$ mà không bị trượt quán tính.

---

## 6. Thuật Toán Nhận Diện Vật Cản Ngẫu Nhiên (Kiểm Định Giả Thuyết Huber)

Ở Lượt 2, có 1 khối vật cản ngẫu nhiên được bốc thăm đặt vào 1 trong 4 vị trí đã đánh dấu sẵn $\{R_0, R_1, R_2, R_3\}$ ở khu vực góc phải và gần đích.

1. **Chiến lược Điểm quan sát `Q` (`STATE_OBSERVE`):**
   * Xe đi theo tuyến chung `ROUTE_COMMON` tới điểm dừng quan sát $Q(x_Q, y_Q)$ nằm ngay trước khu vực phân nhánh.
   * Tại $Q$, xe dừng hẳn và xoay đầu qua một loạt góc nhìn cố định $\{\theta_{\text{scan}, 1}, \theta_{\text{scan}, 2}\}$, thu thập tập gồm $M$ phép đo ToF tĩnh $\{z_1, z_2, \dots, z_M\}$.

2. **Hàm chi phí bền vững Huber (Robust Huber Loss):**
   Với mỗi giả thuyết $k \in \{0, 1, 2, 3\}$ (giả sử vật cản ngẫu nhiên nằm tại vị trí $R_k$), STM32 thực hiện Raycast vào bản đồ có gắn thêm khối hộp $R_k$ để lấy khoảng cách dự đoán $h_{m}^{(k)}$.
   Chi phí của giả thuyết $k$ được tính bằng tổng hàm mất mát Huber trên mọi tia đo $m = 1 \dots M$:
   $$J_k = \sum_{m=1}^{M} \rho_{\delta}\left(z_m - h_m^{(k)}\right)$$
   Trong đó hàm Huber $\rho_\delta(e)$ với ngưỡng $\delta = 0.04\text{ m}$ ($4\text{ cm}$) được định nghĩa:
   $$\rho_\delta(e) = \begin{cases} \frac{1}{2} e^2 & \text{nếu } |e| \le \delta \\ \delta \left(|e| - \frac{1}{2}\delta\right) & \text{nếu } |e| > \delta \end{cases}$$

3. **Quy tắc ra quyết định:**
   $$k^* = \arg\min_{k \in \{0, 1, 2, 3\}} J_k$$
   * Sở dĩ dùng hàm **Huber** thay vì bình phương tối tiểu ($e^2$) là để tránh việc 1 tia ToF bị nhiễu bất thường làm sai lệch toàn bộ tổng điểm.
   * Ngay sau khi tìm được $k^*$, xe nạp tuyến đường nhánh `ROUTE_BRANCH[k*]` và đồng thời **thêm khối hộp $R_{k^*}$ vào danh sách vật cản của EKF**, biến chính vật cản ngẫu nhiên đó thành mốc định vị mới cho đoạn đường về đích!

---

## 7. Thuật Toán Tự Phục Hồi Khi Bị Nhấc Xe (`LIFTED` $\rightarrow$ `RELOC`)

Theo luật thi, khi xe gặp sự cố phải "Chỉnh xe", nhóm được nhấc xe lên và đặt lại ngay sau **vạch chia phần cuối cùng mà cả xe vừa vượt qua (`last_line`)**. Hệ thống xử lý tự động hoàn toàn qua 4 bước:

1. **Theo dõi vạch chia phần liên tục (`last_line` tracker):**
   * Trong lúc chạy ở trạng thái `RUN`, mỗi khi tọa độ đuôi xe $(\hat{x} - 0.039\cos\hat{\theta}, \hat{y} - 0.039\sin\hat{\theta})$ vượt qua phương trình đường thẳng của vạch chia phần thứ $n$, biến `last_line` được cập nhật $n \leftarrow n_{\text{new}}$.
2. **Phát hiện nhấc xe (`LIFTED`):**
   * Khi cả 4 cảm biến hồng ngoại `IR0..IR3` dưới gầm đồng thời mất phản xạ mặt sàn (ADC vượt ngưỡng $> 3200$ liên tiếp trong $40\text{ ms}$), máy trạng thái lập tức chuyển sang `STATE_LIFTED` và ngắt PWM động cơ về $0$.
3. **Định vị lại tại vạch (`RELOC`):**
   * Khi người chơi đặt xe xuống sau vạch `last_line` và bấm nút `START`:
   * Xe đứng yên $1.0\text{ s}$ để đo lại `gyro_bias`.
   * Khởi tạo vị trí hạt giống tại tâm vạch `last_line`: $(x_{\text{seed}}, y_{\text{seed}}, \theta_{\text{seed}}) = \text{Checkpoint}[\text{last\_line}]$.
   * Thực hiện **quét lưới cục bộ (Local Grid Search)** quanh điểm hạt giống trong phạm vi $\Delta y \in [-8\text{ cm}, +8\text{ cm}]$ (dọc theo chiều ngang làn đường) và $\Delta \theta \in [-10^\circ, +10^\circ]$: điểm lưới nào cho tổng sai số Raycast với 5 cảm biến ToF hiện tại nhỏ nhất sẽ được chọn làm Pose khởi tạo mới cho EKF.
4. **Nối lại hành trình:**
   * Tự động tạo một đoạn nối thẳng từ vị trí vừa khớp về Waypoint tiếp theo phía trước vạch `last_line` và tiếp tục chạy về đích.

---

## 8. Bộ Điều Khiển Vận Tốc Bánh Xe Cấp Thấp ($1\text{ kHz}$ PI + Feedforward)

Chạy bên trong ngắt cứng `TIM5` đúng mỗi $1\text{ ms}$ để cưỡng bức vận tốc thực tế của 2 động cơ N20 bám sát lệnh $(v_{L,\text{ref}}, v_{R,\text{ref}})$ bất chấp sụt áp pin hay ma sát mặt sàn:

1. **Bộ lọc thông thấp vận tốc Encoder (Tránh gai đạo hàm):**
   $$v_{f}[k] = \alpha_v v_{f}[k-1] + (1 - \alpha_v) v_{\text{meas}}[k], \quad (\alpha_v = 0.7)$$
2. **Luật điều khiển PI kết hợp Bù trước (Feedforward) và Bù vùng chết ma sát (Deadzone Compensation):**
   $$e[k] = v_{\text{ref}}[k] - v_{f}[k]$$
   $$I[k] = \text{sat}_{I_{\max}}\left(I[k-1] + K_i \cdot e[k] \cdot \Delta t\right)$$
   $$u_{\text{pwm}}[k] = \text{sgn}(v_{\text{ref}}) \cdot u_{\text{deadzone}} + K_{ff} \cdot v_{\text{ref}}[k] + K_p \cdot e[k] + I[k]$$
3. **Chế độ băm xung Slow-Decay (Phanh điện từ trên DRV8833):**
   * Thay vì thả trôi bánh xe (Fast Decay / Coast) làm xe bị trôi theo quán tính khi giảm tốc, cả 2 kênh PWM của mỗi cầu H được điều khiển ở chế độ **Slow Decay (Active Braking)**:
     * Khi $u_{\text{pwm}} \ge 0$: `IN1 = ARR`, `IN2 = ARR - u_pwm`.
     * Khi $u_{\text{pwm}} < 0$: `IN1 = ARR - |u_pwm|`, `IN2 = ARR`.
   * Cách băm xung này giúp quan hệ giữa Duty Cycle và vận tốc bánh xe trở nên tuyến tính tuyệt đối ngay cả khi xe giảm tốc độ đột ngột trước khúc cua.