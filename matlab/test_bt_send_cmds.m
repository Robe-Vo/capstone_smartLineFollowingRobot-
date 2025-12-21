% ======================= test_cmds_bt_device.m =======================
% MATLAB Bluetooth connection to device name: "Behind the scream"
% Send CMD bytes + payload bytes over Bluetooth SPP.
%
% Requirements:
% - MATLAB must support Bluetooth SPP on your OS.
% - Your ESP32 must be running BluetoothSerial SPP with name "Behind the scream".
%
% Notes:
% - "speed 2 bytes" here uses int16 (Hz*100). This must match ESP32 compile flag:
%     -DPROTOCOL_OP_SPEED_I16X100
% - If you use float16 instead, you must replace mkSpeed2B() with float16 encoder.

clear; clc;

DEV_NAME = "Behind the scream122";
CHANNEL  = 1;            % SPP channel; usually 1. If connection fails, try 2..10
ACK      = uint8(hex2dec('20'));

% ---------- helpers ----------
u16le = @(v) uint8([bitand(uint16(v),255), bitand(bitshift(uint16(v),-8),255)]);
i16le = @(v) u16le(typecast(int16(v),'uint16'));
mkSpeed2B = @(hz) i16le(round(hz*100));   % int16 Hz*100

p30 = uint8(0:29);

% ---------- CMD values (must match ESP32 enum) ----------
CMD.GLOBAL_OPERATION = hex2dec('FF');
CMD.GLOBAL_IDLE      = hex2dec('FE');
CMD.PING_MODE        = hex2dec('FD');
CMD.EM_STOP          = hex2dec('FC');

CMD.OP_SPD_FWD = hex2dec('EF');
CMD.OP_SPD_BWD = hex2dec('EE');
CMD.OP_PWM_FWD = hex2dec('ED');
CMD.OP_PWM_BWD = hex2dec('EC');
CMD.OP_BRAKE   = hex2dec('EB');

CMD.MOTOR_EN   = hex2dec('DF');
CMD.MOTOR_DIS  = hex2dec('DE');
CMD.MOTOR_SPD_FWD = hex2dec('DD');
CMD.MOTOR_SPD_BWD = hex2dec('DC');
CMD.MOTOR_PWM_FWD = hex2dec('DB');
CMD.MOTOR_PWM_BWD = hex2dec('DA');
CMD.MOTOR_BRAKE   = hex2dec('D9');

CMD.SERVO_EN   = hex2dec('D8');
CMD.SERVO_DIS  = hex2dec('D7');
CMD.SERVO_WR   = hex2dec('D6');
CMD.SERVO_CENTER = hex2dec('D5');
CMD.SERVO_RD     = hex2dec('D4');

CMD.LINE_RD   = hex2dec('CF');
CMD.ULTRA_RD  = hex2dec('CE');
CMD.MPU_RD    = hex2dec('CD');

CMD.SET_LINE  = hex2dec('BF');
CMD.SET_MPU   = hex2dec('BE');
CMD.SET_ULTRA = hex2dec('BD');
CMD.SET_MOTOR = hex2dec('BC');
CMD.SET_SERVO = hex2dec('BB');
CMD.SET_PID   = hex2dec('BA');

% ---------- test list ----------
tests = {
  "GLOBAL_OPERATION", uint8(CMD.GLOBAL_OPERATION), uint8([])
  "GLOBAL_IDLE",      uint8(CMD.GLOBAL_IDLE),      uint8([])
  "PING_MODE",        uint8(CMD.PING_MODE),        uint8([])
  "EM_STOP",          uint8(CMD.EM_STOP),          uint8([])

  "OP_SPD_FWD",       uint8(CMD.OP_SPD_FWD),       [mkSpeed2B(5.0) i16le(75)]
  "OP_SPD_BWD",       uint8(CMD.OP_SPD_BWD),       [mkSpeed2B(5.0) i16le(75)]
  "OP_PWM_FWD",       uint8(CMD.OP_PWM_FWD),       [u16le(800) i16le(75)]
  "OP_PWM_BWD",       uint8(CMD.OP_PWM_BWD),       [u16le(800) i16le(75)]
  "OP_BRAKE",         uint8(CMD.OP_BRAKE),         uint8([])

  "MOTOR_EN",         uint8(CMD.MOTOR_EN),         uint8([])
  "MOTOR_DIS",        uint8(CMD.MOTOR_DIS),        uint8([])
  "MOTOR_SPD_FWD",    uint8(CMD.MOTOR_SPD_FWD),    mkSpeed2B(3.0)
  "MOTOR_SPD_BWD",    uint8(CMD.MOTOR_SPD_BWD),    mkSpeed2B(3.0)
  "MOTOR_PWM_FWD",    uint8(CMD.MOTOR_PWM_FWD),    u16le(500)
  "MOTOR_PWM_BWD",    uint8(CMD.MOTOR_PWM_BWD),    u16le(500)
  "MOTOR_BRAKE",      uint8(CMD.MOTOR_BRAKE),      uint8([])

  "SERVO_EN",         uint8(CMD.SERVO_EN),         uint8([])
  "SERVO_DIS",        uint8(CMD.SERVO_DIS),        uint8([])
  "SERVO_WR",         uint8(CMD.SERVO_WR),         i16le(75)
  "SERVO_CENTER",     uint8(CMD.SERVO_CENTER),     uint8([])
  "SERVO_RD",         uint8(CMD.SERVO_RD),         uint8([])

  "LINE_RD",          uint8(CMD.LINE_RD),          uint8([])
  "ULTRA_RD",         uint8(CMD.ULTRA_RD),         uint8([])
  "MPU_RD",           uint8(CMD.MPU_RD),           uint8([])

  "SET_LINE",         uint8(CMD.SET_LINE),         p30
  "SET_MPU",          uint8(CMD.SET_MPU),          p30
  "SET_ULTRA",        uint8(CMD.SET_ULTRA),        p30
  "SET_MOTOR",        uint8(CMD.SET_MOTOR),        p30
  "SET_SERVO",        uint8(CMD.SET_SERVO),        p30
  "SET_PID",          uint8(CMD.SET_PID),          p30

  "INVALID_A5",       uint8(hex2dec('A5')),        uint8([])
};

% ---------- connect via Bluetooth ----------
% MATLAB API variants depend on release/OS.
% Try bluetooth() first (newer). If it fails, use bluetoothdev().
bt = [];

try
  bt = bluetooth(DEV_NAME, CHANNEL);
catch
  % fallback for some MATLAB versions:
  try
    bt = bluetoothdev(DEV_NAME, CHANNEL);
  catch
    error("Cannot open Bluetooth SPP to '%s' (channel %d). Pair device first, then try another channel.", DEV_NAME, CHANNEL);
  end
end

pause(0.2);

fprintf("=== Connected to '%s' (channel %d) ===\n", DEV_NAME, CHANNEL);
fprintf("=== Sending %d CMDs ===\n", size(tests,1));

% Some MATLAB objects use write() with uint8, some use fwrite().
% We'll try write() then fallback to fwrite().
useWrite = true;
try
  write(bt, uint8([0]), "uint8");
catch
  useWrite = false;
end

% ---------- send loop ----------
for k = 1:size(tests,1)
  name = tests{k,1};
  cmd  = tests{k,2};
  pl   = tests{k,3};

  pkt = uint8([cmd; pl(:)]);

  if useWrite
    write(bt, pkt, "uint8");
  else
    fwrite(bt, pkt, "uint8");
  end

  pause(0.05);

  % Read optional ACK if ESP32 sends it back on BT.
  gotAck = false;
  rx = uint8([]);
  t0 = tic;

  while toc(t0) < 0.15
    n = 0;
    try
      n = bt.NumBytesAvailable;
    catch
      % some objects do not expose NumBytesAvailable reliably
      n = 0;
    end

    if n > 0
      if useWrite
        r = read(bt, n, "uint8");
      else
        r = fread(bt, n, "uint8");
        r = uint8(r);
      end
      rx = [rx; r(:)]; %#ok<AGROW>
      if any(r == ACK), gotAck = true; end
    end
    pause(0.01);
  end

  fprintf("%-14s cmd=0x%02X payload=%2d  ACK=%d  rx=%d\n", ...
    name, cmd, numel(pl), gotAck, numel(rx));

  pause(0.10);
end

% ---------- cleanup ----------
try
  clear bt;
catch
end
