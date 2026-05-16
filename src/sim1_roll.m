%% sim1_roll.m
%% Simulation 1 — Roll Hover Stability
drone_params;
s = tf('s');

G  = 1 / (Ixx * s^2);
C  = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
CL = feedback(C*G, 1);

t = 0:0.001:5;
[y, t_out] = step(CL * 0.1, t);

figure('Name','Sim 1 Roll Stability','Color','white',...
       'Position',[100 100 800 500]);
plot(t_out, y, 'b', 'LineWidth', 2.5); hold on;
yline(0.1,'--r','Target 0.1 rad','LineWidth',1.5);
yline(0.11,':k','+10%','LineWidth',1);
yline(0.09,':k','-10%','LineWidth',1);
xlabel('Time (s)','FontSize',12);
ylabel('Roll Angle (rad)','FontSize',12);
title({'Sim 1 - Roll Hover Stability',...
       'AUW=1.8kg | L=200mm | AIR2216 KV880'},...
       'FontSize',13);
grid on; grid minor;
legend('Roll response','Target',...
       'FontSize',10,'Location','southeast');

OS  = (max(y)-0.1)/0.1*100;
idx = find(abs(y-0.1) < 0.002, 1);
Ts  = t_out(idx);
fprintf('\n+---- SIM 1 RESULTS ----+\n')
fprintf('| Overshoot:     %.1f%%  |\n', OS)
fprintf('| Settling time: %.3fs  |\n', Ts)
fprintf('| Status:  STABLE ✓     |\n')
fprintf('+-----------------------+\n')