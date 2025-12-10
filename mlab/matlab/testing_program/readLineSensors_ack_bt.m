function readLineSensors_ack_bt()
% Continuously read 5xuint8 line sensors via Bluetooth with ACK.
%
% Protocol per request:
%   PC   -> Robot : 0xEF
%   Robot -> PC   : 0x20 (ACK) + [L1 L2 L3 L4 L5] (5 bytes, uint8)

    %% --- User config ---
    btName    = "Behind the scream";  % Bluetooth device name
    btChannel = 1;                    % RFCOMM channel (change if needed)

    %% --- Connect to Bluetooth ---
    dev = bluetooth(btName, btChannel);
    dev.Timeout = 1;  % seconds
    fprintf("Connected to %s (channel %d)\n", btName, btChannel);

    fprintf("Reading line sensors with ACK. Press Ctrl+C to stop.\n");

    try
        while true
            %% 1) Send command 0xEF (read line)
            write(dev, uint8(hex2dec('EF')), "uint8");

            %% 2) Read ACK byte (0x20)
            ack = [];
            try
                ack = read(dev, 1, "uint8");
            catch
                fprintf("ACK read timeout/error.\n");
                pause(0.05);
                continue;
            end

            if isempty(ack)
                fprintf("No ACK received.\n");
                pause(0.05);
                continue;
            end

            if ack ~= uint8(0x20)
                fprintf("Unexpected ACK byte: 0x%02X (expected 0x20)\n", ack);
                % Optionally: flush remaining bytes here
                pause(0.05);
                continue;
            end

            %% 3) Read 5 bytes of line sensor data
            data = [];
            try
                data = read(dev, 5, "uint8");
            catch
                fprintf("Line data read timeout/error.\n");
                pause(0.05);
                continue;
            end

            if numel(data) ~= 5
                fprintf("Got %d bytes instead of 5 for line data.\n", numel(data));
                pause(0.05);
                continue;
            end

            %% 4) Process / display
            lineVals = double(data(:)).';  % convert to 1x5 double for printing
            fprintf("Line: [%3d %3d %3d %3d %3d]\n", lineVals);

            % Small delay to avoid spamming too fast (tune as you like)
            pause(0.02);
        end

    catch ME
        fprintf("Stopped: %s\n", ME.message);
    end

    %% --- Clean up ---
    clear dev;
    fprintf("Bluetooth connection closed.\n");
end
