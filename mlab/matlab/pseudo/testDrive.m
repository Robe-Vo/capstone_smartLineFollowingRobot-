% MATLAB pseudocode – safety test in IDLE mode (Bluetooth)
clear,clc
btName    = "Behind the scream";
btChannel = 1;
bt = bluetooth(btName, btChannel);    % adjust to your MATLAB version

writeBytes = @(bytes) write(bt, uint8(bytes), "uint8");

%% 1. Ensure robot is in IDLE and drive disabled
% Send "back to IDLE" if needed
writeBytes(hex2dec('FE'));    % optional, if firmware uses FE->IDLE
pause(0.05);

% Disable drive to be sure
writeBytes(hex2dec('E2'));    % DISABLE_DRIVE
pause(0.05);

% Try to send speed command WITHOUT enable
speed_test = 120;
writeBytes([hex2dec('DF') speed_test]);   % forward command
pause(1.0);   % expectation: robot does NOT move

%% 2. Enable drive, then send the same command
writeBytes(hex2dec('E1'));    % ENABLE_DRIVE
pause(0.05);

writeBytes([hex2dec('DF') speed_test]);   % forward command, now robot SHOULD move
pause(2.0);

%% 3. Stop and disable again
writeBytes(hex2dec('DD'));    % STOP motor (brake)
pause(0.5);
writeBytes(hex2dec('E2'));    % DISABLE_DRIVE
pause(0.5);

%% 4. Optional: quick test backward and emergency stop
speed_back = 100;
writeBytes(hex2dec('E1'));          % ENABLE_DRIVE
pause(0.05);
writeBytes([hex2dec('DE') speed_back]);  % backward command
pause(1.0);
writeBytes(hex2dec('DD'));          % emergency stop
pause(0.5);
writeBytes(hex2dec('E2'));          % DISABLE_DRIVE

clear bt;
