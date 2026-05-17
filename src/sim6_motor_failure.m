%% sim6_motor_failure.m
%% Simulation 6 — Motor Failure Analysis
drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

t    = 0:0.001:10;
dt   = 0.001;
t_fail = 5.0;    % motor fails at t=5s

%% Normal motor speeds at hover
w1=ones(size(t))*w_hover;  % Motor 1 FR
w2=ones(size(t))*w_hover;  % Motor 2 FL
w3=ones(size(t))*w_hover;  % Motor 3 RL
w4=ones(size(t))*w_hover;  % Motor 4 RR

%% Motor 4 fails at t=5s — speed drops to zero
w4(t >= t_fail) = w_hover * max(0, 1 - (t(t>=t_fail)-t_fail)/0.5);
% Ramps down to zero in 0.5s

%% Thrust from each motor
T1 = kf * w1.^2;
T2 = kf * w2.^2;
T3 = kf * w3.^2;
T4 = kf * w4.^2;

%% Total thrust and torques
T_total = T1 + T2 + T3 + T4;
T_hover_val = m * g;

%% Roll torque — FR and RR on right, FL and RL on left
%% X frame: motor positions at 45 degrees
%% Roll: (T_right - T_left) * L * cos(45)
tau_roll = ((T1+T4) - (T2+T3)) * L * cos(pi/4);

%% Pitch torque
tau_pitch = ((T1+T2) - (T3+T4)) * L * cos(pi/4);

%% Yaw torque — CW motors (1,3) vs CCW motors (2,4)
tau_yaw = km*w1.^2 - km*w2.^2 + km*w3.^2 - km*w4.^2;

%% Altitude deviation (simplified)
%% When one motor fails, total thrust drops
T_deficit = T_total - T_hover_val;
z_accel   = T_deficit / m;

%% Integrate to get altitude deviation
z_dot = cumtrapz(t, z_accel);
z_dev = cumtrapz(t, z_dot);

%% Roll angle from torque (open loop — no compensation)
roll_accel   = tau_roll / Ixx;
roll_dot     = cumtrapz(t, roll_accel);
roll_dev     = cumtrapz(t, roll_dot);

%% Print results
fail_roll = max(abs(rad2deg(roll_dev(t>t_fail))));
fail_alt  = min(z_dev(t>t_fail));

fprintf('\n+------ SIM 6 RESULTS ------+\n')
fprintf('| Motor failed:  Motor 4 RR  |\n')
fprintf('| Fail time:     %.0f s      |\n', t_fail)
fprintf('| Max roll dev:  %.1f deg    |\n', fail_roll)
fprintf('| Alt drop:      %.2f m     |\n', abs(fail_alt))
fprintf('| Safety:        RTL needed  |\n')
fprintf('+----------------------------+\n')

%% Plot
f6=figure('Name','Sim 6 Motor Failure',...
          'Color','white','Position',[50 50 1200 800]);

subplot(3,2,1);
plot(t,w1/w_hover*100,'b','LineWidth',2); hold on;
plot(t,w2/w_hover*100,'r','LineWidth',2);
plot(t,w3/w_hover*100,'g','LineWidth',2);
plot(t,w4/w_hover*100,'m','LineWidth',2.5);
xline(t_fail,'--k','Motor 4 fails','LineWidth',2,...
      'LabelVerticalAlignment','bottom');
xlabel('Time (s)','FontSize',11);
ylabel('Motor Speed (% hover)','FontSize',11);
title('Motor Speeds — Motor 4 Failure','FontSize',12);
legend('M1 FR','M2 FL','M3 RL','M4 RR (FAILS)',...
       'FontSize',9,'Location','southwest');
ylim([-5 120]); grid on; grid minor;

subplot(3,2,2);
plot(t,T_total,'b','LineWidth',2); hold on;
yline(T_hover_val,'--r','Hover thrust needed','LineWidth',1.5);
xline(t_fail,'--k','','LineWidth',1.5);
xlabel('Time (s)','FontSize',11);
ylabel('Total Thrust (N)','FontSize',11);
title('Total Thrust vs Required','FontSize',12);
legend('Actual thrust','Required for hover','FontSize',9);
grid on; grid minor;

subplot(3,2,3);
plot(t,rad2deg(roll_dev),'b','LineWidth',2); hold on;
xline(t_fail,'--k','Failure','LineWidth',1.5);
yline(0,'--','Color',[0.5 0.5 0.5]);
yline(30,':r','Critical 30°'); yline(-30,':r');
xlabel('Time (s)','FontSize',11);
ylabel('Roll Deviation (deg)','FontSize',11);
title('Roll Angle After Motor Failure','FontSize',12);
grid on; grid minor;

subplot(3,2,4);
plot(t,rad2deg(tau_yaw./Izz.*t),'g','LineWidth',2); hold on;
plot(t,rad2deg(tau_yaw/Izz),'m','LineWidth',2);
xline(t_fail,'--k','Failure','LineWidth',1.5);
xlabel('Time (s)','FontSize',11);
ylabel('Yaw Rate (deg/s)','FontSize',11);
title('Yaw Disturbance After Failure','FontSize',12);
legend('Yaw angle','Yaw rate','FontSize',9);
grid on; grid minor;

subplot(3,2,5);
plot(t,z_dev,'r','LineWidth',2); hold on;
xline(t_fail,'--k','Failure','LineWidth',1.5);
yline(0,'--','Color',[0.5 0.5 0.5]);
xlabel('Time (s)','FontSize',11);
ylabel('Altitude Change (m)','FontSize',11);
title('Altitude Deviation After Failure','FontSize',12);
grid on; grid minor;

subplot(3,2,6);
% Safety assessment
categories = {'Thrust\nLoss','Roll\nDev','Yaw\nDist','Alt\nDrop'};
severity   = [25, min(fail_roll/45*100,100),...
              50, min(abs(fail_alt)/2*100,100)];
colors_b   = [0.9 0.3 0.3; 0.9 0.6 0.2;
              0.9 0.6 0.2; 0.9 0.3 0.3];
b = bar(severity,'FaceColor','flat');
b.CData = colors_b;
set(gca,'XTickLabel',{'Thrust Loss','Roll Dev',...
                       'Yaw Dist','Alt Drop'});
yline(50,'--r','Critical','LineWidth',1.5);
ylabel('Severity (%)','FontSize',11);
title('Failure Impact Assessment','FontSize',12);
ylim([0 110]); grid on;

sgtitle({'Sim 6 — Motor Failure Analysis',...
         'Motor 4 (RR) Fails at t=5s | AUW=1.8kg | L=200mm'},...
         'FontSize',14,'FontWeight','bold');

saveas(f6,[savepath 'Sim6_MotorFailure.png']);
fprintf('\nSaved: Sim6_MotorFailure.png\n')
fprintf('\nSafety conclusion:\n')
fprintf('  Motor failure requires immediate RTL\n')
fprintf('  Failsafe: RTL on motor current drop\n')
fprintf('  Recommend: current monitoring on each ESC\n')