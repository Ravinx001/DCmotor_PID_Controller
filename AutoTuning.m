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

% Define tuning options for pidtune
opts = pidtuneOptions('DesignFocus', 'reference-tracking');

% Automatically tune controllers with options
[controllerP, infoP] = pidtune(motorTF, 'P', opts);
[controllerPI, infoPI] = pidtune(motorTF, 'PI', opts);
[controllerPID, infoPID] = pidtune(motorTF, 'PID', opts);

% Display tuned gains
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

% Closed-loop systems
closedLoopP = feedback(controllerP * motorTF, 1);
closedLoopPI = feedback(controllerPI * motorTF, 1);
closedLoopPID = feedback(controllerPID * motorTF, 1);

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
title('Step Response (0 to 90°) - Automatic Tuning');
xlabel('Time (s)');
ylabel('Position (°)');
grid on;
yline(90, 'k--', '90° Reference', 'LineWidth', 1.2, 'LabelHorizontalAlignment', 'center');
legend('P Controller', 'PI Controller', 'PID Controller', 'Reference');

% Step response metrics
metricsP = stepinfo(targetAngle * closedLoopP);
metricsPI = stepinfo(targetAngle * closedLoopPI);
metricsPID = stepinfo(targetAngle * closedLoopPID);

% Display step response metrics
disp('---- Step Response Metrics ----');
disp('P Controller:');
disp(metricsP);
disp('PI Controller:');
disp(metricsPI);
disp('PID Controller:');
disp(metricsPID);

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
yline(metricsP.Peak, 'y--', sprintf('Overshoot = %.2f%%', metricsP.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
yline(finalPosP, 'b--', sprintf('Final Position = %.1f°', finalPosP), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'left');
title('P Controller Step Response with Metrics');
xlabel('Time (s)');
ylabel('Position (°)');
legend('P Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Final Position', 'Location', 'southeast');
grid on;

% Plot PI controller response with metrics
figure;
plot(time, outputPI, 'c', 'LineWidth', 1); hold on;
ylim([0 120]);
xlim([0 25]);
yline(90, 'k--', '90° Reference', 'LineWidth', '0.75');
xline(metricsPI.RiseTime, 'r--', sprintf('Rise Time = %.2fs', metricsPI.RiseTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(metricsPI.SettlingTime, 'g--', sprintf('Settling Time = %.2fs', metricsPI.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
yline(metricsPI.Peak, 'y--', sprintf('Overshoot = %.2f%%', metricsPI.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
yline(finalPosPI, 'b--', sprintf('Final Position = %.1f°', finalPosPI), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'left');
title('PI Controller Step Response with Metrics');
xlabel('Time (s)');
ylabel('Position (°)');
legend('PI Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Final Position', 'Location', 'south');
grid on;

% Plot PID controller response with metrics
figure;
plot(time, outputPID, 'b', 'LineWidth', 1); hold on;
ylim([0 120]);
xlim([0 10]);
yline(90, 'k--', '90° Reference', 'LineWidth', 0.75);
xline(metricsPID.RiseTime, 'r--', sprintf('Rise Time = %.2fs', metricsPID.RiseTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
xline(metricsPID.SettlingTime, 'g--', sprintf('Settling Time = %.2fs', metricsPID.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LabelHorizontalAlignment', 'center');
yline(metricsPID.Peak, 'y--', sprintf('Overshoot = %.2f%%', metricsPID.Overshoot), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
yline(finalPosPID, 'b--', sprintf('Final Position = %.1f°', finalPosPID), 'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'left');
title('PID Controller Step Response with Metrics');
xlabel('Time (s)');
ylabel('Position (°)');
legend('PID Response', '90° Reference', 'Rise Time', 'Settling Time', 'Overshoot', 'Final Position', 'Location', 'southeast');
grid on;