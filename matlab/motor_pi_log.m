clear s;

% Ket noi STM32
s = serialport("COM4", 115200, "Timeout", 1);
configureTerminator(s, "CR/LF");
flush(s);

data = zeros(0, 6);

disp("Dang ghi du lieu trong 30 giay.");
disp("Chuyen sang Keil, dat test_request = 1 roi Run.");

captureTimer = tic;

try
    while toc(captureTimer) < 30
        if s.NumBytesAvailable > 0
            line = readline(s);

            if ~isempty(line)
                values = sscanf(char(line), ...
                    '%f,%f,%f,%f,%f,%f');

                if numel(values) == 6 && all(isfinite(values))
                    data(end + 1, :) = values.';
                end
            end
        else
            pause(0.005);
        end
    end
catch ME
    clear s;
    rethrow(ME);
end

clear s;

if isempty(data)
    error("Khong nhan du lieu. Kiem tra COM4 va Keil dang Run.");
end

% Chuyen ve don vi thuc
time_s       = (data(:, 1) - data(1, 1)) / 1000;
setpoint_rpm = data(:, 2) / 10;
measured_rpm = data(:, 3) / 10;
pwm_percent  = data(:, 4) / 10;
running      = data(:, 5);
fault        = data(:, 6);

% Chi hien toc do dat khi PI dang hoat dong
reference_plot = setpoint_rpm;
reference_plot(running == 0) = NaN;

% Ve do thi sau khi thu xong
figure('Name', 'Ket qua thu nghiem PI');

subplot(2, 1, 1);
plot(time_s, reference_plot, 'r--', 'LineWidth', 1.5);
hold on;
plot(time_s, measured_rpm, 'b-', 'LineWidth', 1.2);
grid on;
xlabel('Thoi gian (s)');
ylabel('Toc do (RPM)');
legend('Toc do dat khi PI chay', 'Toc do encoder', ...
       'Location', 'best');
title('Dap ung toc do dong co');

subplot(2, 1, 2);
plot(time_s, pwm_percent, 'k-', 'LineWidth', 1.2);
grid on;
xlabel('Thoi gian (s)');
ylabel('PWM (%)');
ylim([0 55]);
title('Tin hieu dieu khien');

% Luu moi lan thu thanh mot file rieng
result = table(time_s, setpoint_rpm, measured_rpm, ...
               pwm_percent, running, fault);

filename = ['motor_pi_' datestr(now, 'yyyymmdd_HHMMSS') '.csv'];
writetable(result, filename);

fprintf('Da nhan %d mau.\n', size(data, 1));
fprintf('Da luu: %s\n', fullfile(pwd, filename));

if ~any(running == 1)
    warning("Co du lieu UART nhung chua ghi duoc luot motor chay.");
end

if any(fault ~= 0)
    warning("Co ma loi PI trong du lieu. Kiem tra cot fault.");
end