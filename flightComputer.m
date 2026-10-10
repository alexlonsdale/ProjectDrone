% Clear old connections and figures
clear t;
close all;

% Connect to the local Python bridge
t = tcpclient('127.0.0.1', 5000);
configureTerminator(t, "LF");

disp("Connected! Streaming dual plots. Close figure to stop.");

% Initialize integration variables (degrees)
anglePitch = 0;
angleRoll  = 0;
angleYaw   = 0;

% Setup live plotting figure with two stacked subplots
fig = figure('Name', 'ESP32 IMU Live Telemetry', 'Position', [100, 100, 900, 700]);

% --- Top Subplot: Estimated Angular Position (Integrated Rates) ---
subplot(2, 1, 1);
hPitchPos = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Pitch');
hRollPos  = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Roll');
hYawPos   = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Yaw');
legend('show', 'Location', 'northwest');
xlabel('Time Elapsed (s)');
ylabel('Position (deg)');
title('Estimated Angular Position (Integrated Gyro Rates)');
grid on;

% --- Bottom Subplot: Measured Angular Velocity (Raw Stream) ---
subplot(2, 1, 2);
hPitchVel = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Pitch Rate');
hRollVel  = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Roll Rate');
hYawVel   = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Yaw Rate');
legend('show', 'Location', 'northwest');
xlabel('Time Elapsed (s)');
ylabel('Velocity (deg/s)');
title('Measured Angular Velocity');
grid on;

startTime = tic;
lastLoopTime = toc(startTime);

try
    while isvalid(fig)
        if t.NumBytesAvailable > 0
            dataLine = readline(t);
            parsedData = sscanf(dataLine, '%f,%f,%f');
            
            if length(parsedData) == 3
                currentTime = toc(startTime);
                dt = currentTime - lastLoopTime; 
                lastLoopTime = currentTime;
                
                % Axis mapping: GX -> Pitch, GY -> Roll, GZ -> Yaw
                pitchRate = parsedData(1);
                rollRate  = parsedData(2);
                yawRate   = parsedData(3);
                
                % Euler Integration
                anglePitch = anglePitch + (pitchRate * dt);
                angleRoll  = angleRoll  + (rollRate * dt);
                angleYaw   = angleYaw   + (yawRate * dt);
                
                % --- Add points to Top Subplot (Position) ---
                addpoints(hPitchPos, currentTime, anglePitch);
                addpoints(hRollPos,  currentTime, angleRoll);
                addpoints(hYawPos,   currentTime, angleYaw);
                
                % --- Add points to Bottom Subplot (Velocity) ---
                addpoints(hPitchVel, currentTime, pitchRate);
                addpoints(hRollVel,  currentTime, rollRate);
                addpoints(hYawVel,   currentTime, yawRate);
                
                % Rescale axes dynamically for both subplots
                timeWindow = [max(0, currentTime - 10), max(10, currentTime + 1)];
                
                subplot(2, 1, 1);
                gca.XLim = timeWindow;
                
                subplot(2, 1, 2);
                gca.XLim = timeWindow;
                
                drawnow limitrate;
            end
        end
    end
catch ME
    clear t;
    disp("Stream closed.");
    if ~contains(ME.message, 'Invalid') && ~contains(ME.message, 'Interrupted')
        rethrow(ME);
    end
end

clear t;
