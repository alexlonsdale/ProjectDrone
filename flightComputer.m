% Clear old connections
clear t;

% Connect to the local Python bridge
t = tcpclient('127.0.0.1', 5000);
configureTerminator(t, "LF");

disp("Connected to ESP32 bridge via TCP! Reading data...");

try
    while true
        if t.NumBytesAvailable > 0
            dataLine = readline(t);
            parsedData = sscanf(dataLine, '%f,%f,%f');
            
            if length(parsedData) == 3
                fprintf('Pitch: %6.2f | Yaw: %6.2f | Roll: %6.2f\n', ...
                    parsedData(1), parsedData(2), parsedData(3));
            end
        end
    end
catch
    clear t;
    disp("Stream closed.");
end
