# Smart Line Following Robot – Summary for ChatGPT

> Repository: https://github.com/Robe-Vo/capstone_smartLineFollowingRobot-

## 1. Project overview

- Capstone project: smart line-following mobile robot, Ackermann steering.
- Target use: AGV-like platform (có thể dùng cho việc cấp reel tới máy SMT, bám line theo sa bàn cố định).
- Control stack:
  - Low-level: firmware trên vi điều khiển (ESP32 / MCU) điều khiển động cơ, servo, đọc cảm biến.
  - High-level: MATLAB (scripts + App Designer) để:
    - Gửi lệnh điều khiển (tốc độ + góc lái, IDLE, v.v.).
    - Thu thập dữ liệu (tốc độ, sensor IR, encoder, v.v.).
    - Thực hiện điều khiển On/Off, PID, FSM theo “road/path”.

## 2. Repository layout (theo cấu trúc hiện tại)

- `/low_level`
  - Dùng cho code C/C++ chạy trên robot (main.cpp và các file liên quan).
  - Nhiệm vụ chính:
    - Giao tiếp Bluetooth SPP (device name: `"Behind the scream"`, channel 1).
    - Điều khiển:
      - DC motor drive (tốc độ).
      - Servo Ackermann (góc lái).
    - Xử lý giao thức khung lệnh (byte-based protocol).
    - Đọc:
      - Line sensor (5 x uint8).
      - Encoder (đo vị trí / quãng đường / tốc độ).
  - Không đủ dữ liệu để xác minh chính xác danh sách file hiện có trong thư mục này qua GitHub (viewer đang lỗi).

- `/mlab`
  - MATLAB/Simulink code.
  - Nhiệm vụ chính (từ các phiên làm việc trước):
    - Chương trình On/Off line-following (ví dụ: `OnOff_program_fsm_struct.m`).
    - Chương trình PID line-following đa trạng thái đường (ví dụ: `PID_program_roadStates.m`).
    - Hàm giao tiếp Bluetooth (gửi/nhận frame với robot).
    - Script vẽ/ghi log (IR, encoder, góc lái, v.v.).
  - Tên file cụ thể có thể khác; cần đồng bộ lại nếu không trùng.

## 3. Hardware & mechanics (tóm tắt)

- Cấu trúc cơ khí:
  - Robot dùng cơ cấu lái Ackermann (servo điều khiển góc bánh trước).
  - Động cơ DC drive (ít nhất 1 động cơ chính kéo xe).
- Cảm biến:
  - Dãy line sensor TCRT5000 (5 cảm biến, trả về 5 x uint8).
  - Encoder (ví dụ: IE2-512) đo xung theo quãng đường / góc bánh.
- Servo:
  - Góc điều khiển sử dụng thang 0–180 (logic code thường dùng khoảng ~55–115 cho giới hạn cơ khí).
- Nguồn:
  - Phân tách nguồn điều khiển và nguồn động lực (DC motor) để tránh sụt áp do suất điện động khi dừng động cơ.

## 4. Software control overview

### 4.1. Low-level firmware (main.cpp – trên robot)

Chức năng chính:

- Khởi tạo:
  - UART/Bluetooth SPP server.
  - PWM cho động cơ DC.
  - PWM / servo driver cho góc lái.
  - ADC / digital input cho line sensor.
  - Encoder (interrupt / timer).

- Vòng lặp chính:
  - Lắng nghe byte đầu khung từ Bluetooth.
  - Switch–case trên mã lệnh (ví dụ: `0xF1`, `0xFF`, `0xEF`, `0xDB`).
  - Đọc thêm payload nếu cần (tốc độ, góc lái, v.v.).
  - Cập nhật:
    - Tốc độ động cơ (PWM duty).
    - Góc servo (Ackermann).
    - Trả dữ liệu line sensor theo lệnh yêu cầu.

- Các mode cơ bản:
  - IDLE: motor dừng, servo giữ/gắn về 1 góc an toàn.
  - Steering test: nhận trực tiếp giá trị góc servo raw từ PC để test.
  - Line-sensor streaming: gửi về dãy 5 byte IR cho MATLAB để xử lý PID / On/Off.

### 4.2. MATLAB side

- Gửi lệnh:
  - Gửi 1 byte command ID + payload qua Bluetooth.
  - Đồng bộ với protocol ở `main.cpp`.
- Điều khiển:
  - On/Off line-following:
    - Tính error từ dãy 5 sensor.
    - Ánh xạ error sang góc lái (Ackermann) với giới hạn min/max.
  - PID line-following:
    - Tính error analog/digital.
    - PID trên error -> góc lái.
    - Chọn tập tham số PID theo từng “road segment” (thẳng, cong, T-junction, v.v.).
- FSM:
  - State machine theo road / path (LINE_1, CURVE_R500, …).
  - Chuyển trạng thái dựa trên:
    - Điều kiện IR.
    - Encoder count.

## 5. Communication protocol (byte-level)

### 5.1. Bluetooth link

- Transport: Bluetooth SPP (Serial Port Profile).
- Device name: `"Behind the scream"`.
- Channel: `1`.
- Dạng giao tiếp: khung lệnh bắt đầu bằng 1 byte command ID, sau đó là payload (nếu có).

### 5.2. Command IDs hiện đang sử dụng

Các mã lệnh và khung đã được sử dụng / thiết kế trong các buổi làm việc (được tổng hợp vào CSV riêng: `tx_protocol_main_cpp.csv`):

1. `0xF1` – Gửi tốc độ + góc lái
   - Hướng: PC → Robot.
   - Khung:
     - `[0xF1] [speed] [steer_angle_raw]`
     - `speed`: `uint8`, duty / command tốc độ động cơ drive.
     - `steer_angle_raw`: `uint8`, giá trị điều khiển servo (thang raw 0–180 như trong code).
   - Vai trò: lệnh chính để điều khiển robot chạy bám line (On/Off hoặc PID) từ MATLAB.

2. `0xFF` – Chuyển robot về trạng thái IDLE
   - Hướng: PC → Robot.
   - Khung:
     - `[0xFF]`
   - Hành vi mong đợi:
     - Motor về 0% duty / dừng.
     - Giữ hoặc đưa servo về góc trung tâm (offset).

3. `0xEF` – Yêu cầu frame line sensor
   - Hướng: PC → Robot.
   - Khung gửi:
     - `[0xEF]`
   - Robot trả về:
     - `[0x20] [s1] [s2] [s3] [s4] [s5]`
       - `0x20`: byte ACK.
       - `s1..s5`: `uint8`, giá trị sensor tuyến tính từ trái sang phải.

4. `0xDB` – Gửi trực tiếp góc lái dạng 16-bit
   - Hướng: PC → Robot.
   - Khung gửi:
     - `[0xDB] [angle_hi] [angle_lo]`
       - `angle_hi`, `angle_lo`: 2 byte tạo thành `uint16`:
         - `steer_angle = (uint16_t(hi) << 8) | uint16_t(lo);`
   - Dùng cho:
     - Test servo độc lập (không dùng PID/OnOff trên robot).
     - Cho phép gửi góc lái với độ phân giải cao hơn (16-bit).

5. `0x20` – ACK từ robot
   - Hướng: Robot → PC.
   - Xuất hiện ít nhất trong phản hồi của lệnh `0xEF` (line sensor).
   - Có thể được dùng chung làm ACK cho các lệnh khác (tùy cách cài trong `main.cpp`).

> Các mã lệnh trên đã được xác định từ code/thảo luận trước. Nếu có thêm lệnh mới trong `main.cpp`, cần bổ sung vào `tx_protocol_main_cpp.csv`.

## 6. Line sensor & geometry (tóm tắt)

- Số lượng: 5 cảm biến phản xạ.
- Dữ liệu: mỗi cảm biến trả về 1 byte (0–255) sau khi đọc/chuẩn hóa.
- Thuật toán error (MATLAB):
  - Từ dãy 5 giá trị, tính error lateral so với tâm line (có thể sử dụng weighted-average hoặc xấp xỉ đa thức).
- Đặc trưng line:
  - Độ dày line đã được khảo sát; sai số tối đa cho phép thường ~ ±(9–12) mm tính từ mép line (độ dày tính từ tâm là 13 mm, theo thống nhất trước đó).
  - Không đủ dữ liệu để xác minh đầy đủ toàn bộ thông số hình học (khoảng cách sensor, bán kính lane, v.v.) từ GitHub.

## 7. Usage notes for future ChatGPT sessions

Khi ChatGPT làm việc với repo/proj này, ưu tiên:

1. **Không đổi giao thức lệnh**:
   - Mã lệnh và format khung phải khớp `tx_protocol_main_cpp.csv`.
   - Nếu cần thêm lệnh mới -> đề xuất rõ ràng, giữ nguyên các lệnh cũ.

2. **Không đổi pinout / giới hạn phần cứng** trừ khi user yêu cầu:
   - Giữ nguyên các giá trị như min/max servo angle, giới hạn tốc độ, v.v.

3. **MATLAB – Bluetooth**:
   - Dùng `Bluetooth("Behind the scream", 1)` hoặc wrapper tương đương.
   - Đảm bảo đọc/ghi đúng số byte mỗi khung.

4. **Điều chỉnh PID / On/Off**:
   - Không thay đổi cấu trúc FSM tổng thể nếu user không yêu cầu.
   - Nếu cần tối ưu, chỉ đề xuất chỉnh Kp, Ki, Kd hoặc mapping error → góc.

5. **Ghi rõ giả thiết**:
   - Nếu đề xuất thêm tính năng mới (command ID mới, state mới, v.v.) phải ghi rõ đó là đề xuất, không mặc định là code hiện tại.

## 8. Open points / TODO (cần user cập nhật)

Các mục dưới đây hiện **Không đủ dữ liệu để xác minh** chỉ từ GitHub, cần người dùng bổ sung trực tiếp trong file:

1. Đường dẫn chính xác tới `main.cpp` trong thư mục `/low_level` (và tên các file liên quan).
2. Danh sách đầy đủ tất cả command ID đang được xử lý trong `main.cpp` (ngoài `0xF1`, `0xFF`, `0xEF`, `0xDB`).
3. Thông số chính xác:
   - Line width, khoảng cách giữa các sensor, cao độ sensor so với mặt đường, bán kính các đoạn cong trên sa bàn.
4. Danh sách script MATLAB thực tế trong `/mlab` (tên file, mục đích).
5. Tham số PID / On-Off hiện đang dùng cho từng “road state”.

Cập nhật trực tiếp các mục trên trong file này để các phiên ChatGPT sau có đủ ngữ cảnh.
