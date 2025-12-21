#include "state.hpp"

// Params objects
Network net;
Controller::State controller_params;  
Controller::Cfg pid_params;  
Mode robotMode = Mode::IDLE;

// Define globals declared extern in header
Idle_values  idle_params;
Op_RX_values op_params;
Op_TX_values op_signals;

void IDLE_init()
{
  // Reset struct fields to defaults
  idle_params.motor_isWorking   = false;
  idle_params.motor_mode        = 0;

  idle_params.servo_isWorking   = false;
  idle_params.servo_writeAngle  = SERVO_MID_DEG;

  idle_params.line_signals      = nullptr;
  idle_params.ultra_signal      = 0;
  idle_params.mpu_signal        = nullptr;

  // Hardware-safe actions for IDLE entry
  Drive::brake();
  Steer::disable();

  // Disable ISR of ultra-sonic
  ultra_disable_isr();
}

void OPERATION_init()
{
  // Reset RX control state (what PC commands will update)
  op_params.direction      = true;   // default forward
  op_params.useController  = true;   // default PID
  op_params.isRunning      = false;  // safer: do not run until first OP cmd
  op_params.speed          = 0.0f;
  op_params.pwm            = 0;

  // Reset telemetry cache
  op_signals.line_signals  = nullptr;
  op_signals.ultra_signal  = 0;
  for (int i = 0; i < 5; ++i) op_signals.mpu_signal[i] = 0;

  // Hardware-safe actions for OP entry
  Drive::brake();
  Steer::enable();
  Steer::writeMidAngle();

  // Optional: reset PID internal state when entering OP
  Controller::reset(controller_params); // only if you have a pid_state object

  // Enable interrupt
  ultra_enable_isr();
}

// OPERATION_action
void OPERATION_action()
{
  // run only in OPERATION
  if (robotMode != Mode::OPERATION) return;

  static uint32_t t_last_ctrl_ms = 0;
  static uint32_t t_last_io_ms   = 0;

  const uint32_t now = (uint32_t)millis();

  // ============================================================
  // 1) CONTROL TICK (e.g., 10ms)
  // ============================================================
  if ((uint32_t)(now - t_last_ctrl_ms) >= (uint32_t)TS_CONTROLLER_MS)
  {
    t_last_ctrl_ms = now;

    // Update encoder speed estimate
    Encoder::calculate_vec((uint16_t)TS_CONTROLLER_MS);
    const float meas_hz = Encoder::get_vec_hz();

    // Steering always follows current command angle (you set it in OP handlers)
    // If you store angle in another variable, change here accordingly.
    // For OP commands, angle is typically kept in a global (e.g., op_angle_deg).
    // If you don't have one, remove this line.
    // Steer::writeAngle(op_angle_deg);

    // Drive control
    if (!op_params.isRunning)
    {
      Drive::brake();
    }
    else
    {
      if (op_params.useController)
      {
        // PID branch:
        // Not executed here because Controller API/state objects are not provided in this snippet.
        // Safe behavior: brake until PID wiring is completed.
        (void)meas_hz;
        Drive::brake();
      }
      else
      {
        // PWM branch: apply op_params.pwm directly
        uint16_t pwm = op_params.pwm;
        if (pwm > 2047) pwm = 2047; // 11-bit saturation
        Drive::setPWM(pwm, op_params.direction);
      }
    }
  }

  // ============================================================
  // 2) IO / TELEMETRY DATA TICK (e.g., 50ms)
  // ============================================================
  if ((uint32_t)(now - t_last_io_ms) >= (uint32_t)TS_COMMUNICATION_MS)
  {
    t_last_io_ms = now;

    // Cache sensors for telemetry
    op_signals.line_signals = line_readSignals();
    op_signals.ultra_signal = ultra_getSignal();

    // If you later implement MPU read, fill op_signals.mpu_signal[] here.
  }
}


// Setup robot
void robot_init()
{
    // Setup system
    Encoder::setup();
    Drive::setup();
    Steer::setup();
    sensors_setup();

    // Setup
    Protocol::initDescriptorTable();
    Dispatch::init();


    // Setup state
    robotMode = Mode::IDLE;
    IDLE_init();
    OPERATION_init();
}

// Robot communication
void robot_communication(void* pv)
{
  (void)pv;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TS_COMMUNICATION_MS);

  for(;;)
  {
    vTaskDelayUntil(&last, period);
    Protocol::RxFrame f;
    int n = 0;

    // Read RX data
    while (n < 20 && Protocol::tryRead(net, f))  // cap to avoid long loops
    {
      if (g_stateMutex) xSemaphoreTake(g_stateMutex, portMAX_DELAY);
      Dispatch::run(f); // should include mode gating inside Dispatch::run
      if (g_stateMutex) xSemaphoreGive(g_stateMutex);
      n++;
    }

    // Read signals
    op_signals.line_signals = line_readSignals();
    op_signals.ultra_signal = ultra_getSignal();
    // op_signals.mpu_signal = {0,0,0,0,0}; // not available yet
    op_signals.count = Encoder::get_count();
    op_signals.speed = Encoder::get_iir_vec_hz();
    
    uint8_t* rx_buffer;

    // Package it
    Protocol::package_op_signals(rx_buffer,
                                 op_signals.line_signals,
                                 op_signals.ultra_signal,
                                 op_signals.mpu_signal,
                                 op_signals.count,
                                 op_signals.speed);
  }
}


// Robot calculation
void robot_calculation(void* pv)
{
  (void)pv;
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TS_CONTROLLER_MS);
  const float dt_s = (float)TS_CONTROLLER_MS / 1000.0f;


  for (;;)
  {
    vTaskDelayUntil(&last, period);

    if (robotMode != Mode::OPERATION) {
      // Safety: keep motor stopped if not in OPERATION
      Drive::brake();
      continue;
    }

    // Measure speed (interrupt-driven counts are consumed inside library)
    Encoder::calculate_vec((uint16_t)TS_CONTROLLER_MS);
    const float meas_hz = Encoder::get_vec_hz();

    // Copy command state atomically (handlers may update op_params)
    Op_RX_values local;
    if (g_stateMutex) xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    local = op_params;
    if (g_stateMutex) xSemaphoreGive(g_stateMutex);

    if (!local.isRunning) {
      Drive::brake();
      continue;
    }

    uint16_t duty = 0;

    if (local.useController) {
      // TODO: replace controller_state/controller_cfg with your real globals.
      // extern Controller::State controller_params;
      // extern Controller::Cfg   controller_cfg;
      duty = Controller::update(controller_params, pid_params,
                                local.speed, meas_hz, dt_s);
    } else {
      duty = local.pwm;
    }

    if (duty > 2047) duty = 2047;
    Drive::setPWM(duty, local.direction);
  }

}
