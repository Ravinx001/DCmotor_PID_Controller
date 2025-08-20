% DC motor parameters
momentInertia = 0.01; % kg·m^2
dampingCoeff = 0.1; % N·m·s
torqueConst = 0.01; % Torque and back EMF constant
armatureResistance = 1; % Ohm
armatureInductance = 0.5; % H

% Transfer function from voltage to angular position
numTF = [torqueConst];
denTF = conv([1 0], [armatureInductance*momentInertia, armatureInductance*dampingCoeff + armatureResistance*momentInertia, armatureResistance*dampingCoeff + torqueConst^2]);
motorTF = tf(numTF, denTF);

% Automatically tune controllers for baseline gains
[controllerP, ~] = pidtune(motorTF, 'P');
[controllerPI, ~] = pidtune(motorTF, 'PI');
[controllerPID, ~] = pidtune(motorTF, 'PID');

% Display automatic gains
disp('Automatically Tuned Controller Gains:');
fprintf('P Controller:\n');
fprintf('  Kp = %.4f\n', controllerP.Kp);
fprintf('PI Controller:\n');
fprintf('  Kp = %.4f\n', controllerPI.Kp);
fprintf('  Ki = %.4f\n', controllerPI.Ki);
fprintf('PID Controller:\n');
fprintf('  Kp = %.4f\n', controllerPID.Kp);
fprintf('  Ki = %.4f\n', controllerPID.Ki);
fprintf('  Kd = %.4f\n', controllerPID.Kd);

% Manual tuning gains
gainKpP = controllerP.Kp * 0.9;      % Reduced to minimize overshoot
gainKpPI = controllerPI.Kp * 1.85;    % Adjusted for faster response with stability
gainKiPI = controllerPI.Ki * 1.3;    % Reduced to minimize overshoot
gainKpPID = controllerPID.Kp * 1.15;  % Matches automatic Kp for balanced response
gainKiPID = controllerPID.Ki * 0.01;  % Low to minimize overshoot, ensures zero error
gainKdPID = controllerPID.Kd * 1.4;  % Increased damping for less overshoot and settling time

% Create controller transfer functions
controllerPManual = pid(gainKpP);
controllerPIManual = pid(gainKpPI, gainKiPI);
controllerPIDManual = pid(gainKpPID, gainKiPID, gainKdPID);

% Closed-loop systems
closedLoopP = feedback(controllerPManual * motorTF, 1);
closedLoopPI = feedback(controllerPIManual * motorTF, 1);
closedLoopPID = feedback(controllerPIDManual * motorTF, 1);

% Step input: 90 degrees
targetAngle = 90;
time = 0:0.01:25;

% Simulate step responses
[outputP, ~] = step(targetAngle * closedLoopP, time);
[outputPI, ~] = step(targetAngle * closedLoopPI, time);
[outputPID, ~] = step(targetAngle * closedLoopPID, time);

% Plot combined step responses
figure;
plot(time, outputP, 'm', time, outputPI, 'c', time, outputPID, 'b', 'LineWidth', 1.5);
xticks(0:1:25);
yticks(0:10:120);
title('Step Response (0 to 90°) - Manual Tuning');
xlabel('Time (s)');
ylabel('Position (°)');
grid on;
yline(90, 'k--', '90° Reference', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'center');
legend('P Controller', 'PI Controller', 'PID Controller', 'Reference');

% Step response metrics
metricsP = stepinfo(targetAngle * closedLoopP);
metricsPI = stepinfo(targetAngle * closedLoopPI);
metricsPID = stepinfo(targetAngle * closedLoopPID);

% Display manual gains and metrics
fprintf('\n--- Manually Tuned Controller Gains ---\n');
fprintf('P: Kp = %.4f\n', gainKpP);
fprintf('PI: Kp = %.4f, Ki = %.4f\n', gainKpPI, gainKiPI);
fprintf('PID: Kp = %.4f, Ki = %.4f, Kd = %.4f\n', gainKpPID, gainKiPID, gainKdPID);
fprintf('\n--- Step Response Metrics ---\n');
disp('P Controller:'); disp(metricsP);
disp('PI Controller:'); disp(metricsPI);
disp('PID Controller:'); disp(metricsPID);

% Final settling values
finalPosP = outputP(end);
finalPosPI = outputPI(end);
finalPosPID = outputPID(end);

% Plot P controller response with metrics
figure;
plot(time, outputP, 'm', 'LineWidth', 1); hold on;
ylim([0 120]);
xlim([0 10]);
yline(90, 'k--', '90° Reference', 'LineWidth', 0.75);
xline(metricsP.RiseTime, 'r--', sprintf('Rise Time = %.2fs', metricsP.RiseTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(metricsP.SettlingTime, 'g--', sprintf('Settling Time = %.2fs', metricsP.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
yline(metricsP.Peak, 'r--', sprintf('Overshoot = %.2f%%', metricsP.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
title('P Controller Step Response with Metrics - Manual Tuning');
xlabel('Time (s)');
ylabel('Position (°)');
legend('P Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Location', 'southeast');
grid on;

% Plot PI controller response with metrics
figure;
plot(time, outputPI, 'c', 'LineWidth', 1); hold on;
ylim([0 120]);
xlim([0 25]);
yline(90, 'k--', '90° Reference', 'LineWidth', 0.75);
xline(metricsPI.RiseTime, 'r--', sprintf('Rise Time = %.2fs', metricsPI.RiseTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(metricsPI.SettlingTime, 'g--', sprintf('Settling Time = %.2fs', metricsPI.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
yline(metricsPI.Peak, 'r--', sprintf('Overshoot = %.2f%%', metricsPI.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
title('PI Controller Step Response with Metrics - Manual Tuning');
xlabel('Time (s)');
ylabel('Position (°)');
legend('PI Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Location', 'south');
grid on;

% Plot PID controller response with metrics
figure;
plot(time, outputPID, 'b', 'LineWidth', 1); hold on;
ylim([0 120]);
xlim([0 10]);
yline(90, 'k--', '90° Reference', 'LineWidth', 0.75);
xline(metricsPID.RiseTime, 'r--', sprintf('Rise Time = %.2fs', metricsPID.RiseTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(metricsPID.SettlingTime, 'g--', sprintf('Settling Time = %.2fs', metricsPID.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
yline(metricsPID.Peak, 'r--', sprintf('Overshoot = %.2f%%', metricsPID.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
title('PID Controller Step Response with Metrics - Manual Tuning');
xlabel('Time (s)');
ylabel('Position (°)');
legend('PID Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Location', 'southeast');
grid on;