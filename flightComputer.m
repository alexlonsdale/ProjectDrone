% Clear any existing serial connections to free up the port
clear s;

% --- CONFIGURATION ---
% Update portName based on your OS:
% Windows: "COM3" (check Device Manager)
% Linux:   "/dev/ttyUSB0" or "/dev/ttyACM0"
% Mac:     "/dev/cu.usbserial-XXXX"
portName = "/dev/ttyUSB0"; 
baudRate = 115200;
% ---------------------

% Initialize serial port connection
s = serialport(portName, baudRate);
configureTerminator(s, "LF"); % Match ESP32's Serial.println() newline terminator

disp("Listening for serial data from ESP32... Press Ctrl+C in MATLAB to stop.");

try
    while true
        if s.NumBytesAvailable > 0
            % Read the incoming comma-separated text line
            dataLine = readline(s);

            % Parse the three float values (Pitch, Yaw, Roll rates)
            parsedData = sscanf(dataLine, '%f,%f,%f');

            if length(parsedData) == 3
                pitchRate = parsedData(1);
                yawRate   = parsedData(2);
                rollRate  = parsedData(3);

                % Display decoded values cleanly in the command window
                fprintf('Pitch: %6.2f deg/s | Yaw: %6.2f deg/s | Roll: %6.2f deg/s\n', ...
                    pitchRate, yawRate, rollRate);
            else
                % Catches boot messages or minor text glitches safely without crashing
                fprintf('Ignored / Raw: %s\n', strtrim(dataLine));
            end
        end
    end
catch ME
    % Ensure the serial port object is cleared if you manually break the loop
    clear s;
    disp("Serial port closed.");
    if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle') && ~contains(ME.message, 'Interrupted')
        rethrow(ME);
    end
end
