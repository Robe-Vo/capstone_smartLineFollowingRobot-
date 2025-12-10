% === SERVO TEST (IDLE MODE, 55 -> 110 -> 55) ===
clear,clc
btName    = "Behind the scream";
btChannel = 1;
bt = bluetooth(btName, btChannel);

writeBytes = @(bytes) write(bt, uint8(bytes), "uint8");

% 1. Ensure IDLE mode
writeBytes(hex2dec('FE'));   % back to IDLE (if firmware uses FE->IDLE)
pause(0.05);

% 2. Enable steering (optional: send custom command if you have one)
%    Here we only use 0xDB to set angle.

% Helper: send angle in degrees through 0xDB
sendAngleDeg = @(ang) ...
    ( ...
      writeBytes([ ...
          hex2dec('DB'), ...
          bitshift(uint16(ang), -8), ...  % hi
          bitand(uint16(ang), 255) ...    % lo
      ]) ...
    );

% 3. Sweep up: 55 → 110 degrees
for ang = 55:5:110
    sendAngleDeg(ang);
    pause(0.3);   % wait for servo to settle
end

% 4. Sweep down: 110 → 55 degrees
for ang = 110:-5:55
    sendAngleDeg(ang);
    pause(0.3);
end

% 5. Return to center if you want (e.g. 90 deg)
sendAngleDeg(90);
pause(0.3);

clear bt;
