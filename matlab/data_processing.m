% Define serial port and baud rate
COM_PORT = '/dev/cu.usbserial-B0026GIZ'; % Update this to your actual COM port
BAUD_RATE = 115200; % Update this to your actual baud rate

% Create serial port object
try
    s = serialport(COM_PORT, BAUD_RATE);
catch
    error("Failed to open serial port. Check COM port and baud rate");
end

cleanupObj = onCleanup(@()cleanupSerialPort(s));

% Configure the serial port to continuously read data
configureTerminator(s, 'CR'); % Assuming newline character terminates data

% Prepare figures for plotting
figure;
subplot(3,1,1);
title('Temperature Data');
xlabel('Time');
ylabel('Temperature (C)');
hold on;
tempLine = animatedline('Color', 'b', 'DisplayName', 'Temp', 'LineWidth', 2);
legend show;


subplot(3,1,2);
title('Accelerometer Data');
xlabel('Time');
ylabel('Acceleration (g)');
hold on;
accXLine = animatedline('Color', 'r', 'DisplayName', 'X-axis', 'LineWidth', 2);
accYLine = animatedline('Color', 'g', 'DisplayName', 'Y-axis', 'LineWidth', 2);
accZLine = animatedline('Color', 'b', 'DisplayName', 'Z-axis', 'LineWidth', 2);
legend show;

subplot(3,1,3);
title('Gyroscope Data');
xlabel('Time');
ylabel('Angular Velocity (degrees/s)');
hold on;
gyroXLine = animatedline('Color', 'r', 'DisplayName', 'X-axis', 'LineWidth', 2);
gyroYLine = animatedline('Color', 'g', 'DisplayName', 'Y-axis', 'LineWidth', 2);
gyroZLine = animatedline('Color', 'b', 'DisplayName', 'Z-axis', 'LineWidth', 2);
legend show;

% Initialise Timer
startTime = datetime('now');

% Read and plot data continuously
flush(s); % Clear the buffer
while true
    data = readline(s); % Read a line of data
    C = strsplit(data, ','); % Split the data by comma
    
    % Check if data has 6 elements (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
    if numel(C) == 7
        % Convert string data to numerical
        temperature = str2double(C{1});
        accX = str2double(C{2});
        accY = str2double(C{3});
        accZ = str2double(C{4});
        gyroX = str2double(C{5});
        gyroY = str2double(C{6});
        gyroZ = str2double(C{7});
        
        % Calculate elapsed time in seconds
        currentTime = datetime('now');
        elapsedTime = seconds(currentTime - startTime);
        
        % Add points to the lines
        addpoints(tempLine, elapsedTime, temperature);
        addpoints(accXLine, elapsedTime, accX);
        addpoints(accYLine, elapsedTime, accY);
        addpoints(accZLine, elapsedTime, accZ);
        addpoints(gyroXLine, elapsedTime, gyroX);
        addpoints(gyroYLine, elapsedTime, gyroY);
        addpoints(gyroZLine, elapsedTime, gyroZ);
        
        % Update the graphs to show only the last 20 seconds of data
        xlim(subplot(3,1,1), [max(0, elapsedTime-20), max(20, elapsedTime)]);
        xlim(subplot(3,1,2), [max(0, elapsedTime-20), max(20, elapsedTime)]);
        xlim(subplot(3,1,3), [max(0, elapsedTime-20), max(20, elapsedTime)]);
        
        % Update the graphs
        drawnow;
    end
end

% Cleanup

% Define the cleanup function
function cleanupSerialPort(s)
    if ~isempty(s) && isvalid(s)
        % Close the serial port
        delete(s);
        % Display message
        disp('Serial port closed and object cleared.');
    end
    clear 
    clc
end
