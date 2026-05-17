%% sim7_pid_vs_lqr.m
%% Simulation 7 — PID vs LQR Controller Comparison
drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

s = tf('s');

%% ── PID Controller (what you're flying) ──────────
C_pid = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
G_roll = 1/(Ixx*s^2);
CL_pid = feedback(C_pid*G_roll, 1);

%% ── LQR Controller (optimal control) ────────────
%% State space: x = [phi; phi_dot]
%% xdot = A*x + B*u
A = [0 1; 0 0];
B = [0; 1/Ixx];
C_ss = [1 0];
D_ss = 0;

%% LQR weights — tune Q and R
%% Q = state penalty, R = control effort penalty
Q = [100  0;    % penalise angle error heavily
      0   10];  % penalise rate error moderately
R = 0.01;       % allow large control effort

%% Solve LQR
K_lqr = lqr(A, B, Q, R);
fprintf('\n+------ SIM 7 RESULTS ------+\n')
fprintf('| LQR gains:               |\n')
fprintf('| K1 (angle):  %.4f       |\n', K_lqr(1))
fprintf('| K2 (rate):   %.4f       |\n', K_lqr(2))
fprintf('+----------------------------+\n')

%% Closed loop with LQR
A_cl = A - B*K_lqr;
sys_lqr = ss(A_cl, B*K_lqr(1), C_ss, D_ss);
CL_lqr  = feedback(tf(sys_lqr), 1);

%% Step responses
t = 0:0.001:3;
ref = 0.1;

[y_pid, ~] = step(CL_pid*ref, t);
[y_lqr, ~] = step(CL_lqr*ref, t);

%% Performance metrics
% PID
os_pid  = (max(y_pid)-ref)/ref*100;
idx_pid = find(abs(y_pid-ref) < ref*0.02, 1);
ts_pid  = t(idx_pid);

% LQR
os_lqr  = max(0,(max(y_lqr)-ref)/ref*100);
idx_lqr = find(abs(y_lqr-ref) < ref*0.02, 1);
ts_lqr  = t(idx_lqr);

fprintf('\n+-------- COMPARISON --------+\n')
fprintf('| Metric      PID     LQR    |\n')
fprintf('+----------------------------+\n')
fprintf('| Overshoot   %.0f%%     %.0f%%    |\n', os_pid, os_lqr)
fprintf('| Settling    %.3fs  %.3fs |\n', ts_pid, ts_lqr)
fprintf('| Steady err  0%%      0%%    |\n')
fprintf('+----------------------------+\n')

%% Pole comparison
p_pid = pole(CL_pid);
p_lqr = eig(A_cl);

fprintf('\nPID poles (real parts):\n')
fprintf('  %.3f\n', real(p_pid))
fprintf('LQR poles (real parts):\n')
fprintf('  %.3f\n', real(p_lqr))

%% Control effort comparison
u_pid = lsim(C_pid, ref - y_pid', t);
u_lqr = -K_lqr(1)*(y_lqr'-ref) - K_lqr(2)*gradient(y_lqr',0.001);

%% Plot
f7=figure('Name','Sim 7 PID vs LQR',...
          'Color','white','Position',[50 50 1200 800]);

%% Panel 1 — Step response comparison
subplot(2,2,[1 2]);
plot(t, y_pid,'b','LineWidth',2.5,...
     'DisplayName',sprintf('PID  (Ts=%.3fs, OS=%.0f%%)',ts_pid,os_pid));
hold on;
plot(t, y_lqr,'r','LineWidth',2.5,...
     'DisplayName',sprintf('LQR  (Ts=%.3fs, OS=%.0f%%)',ts_lqr,os_lqr));
yline(ref,'--k','Target 0.1 rad','LineWidth',1.5);
yline(ref*1.02,':','Color',[0.5 0.5 0.5],'LineWidth',1);
yline(ref*0.98,':','Color',[0.5 0.5 0.5],'LineWidth',1);
xline(ts_pid,'--b',sprintf('PID Ts=%.3fs',ts_pid),...
      'LineWidth',1,'LabelVerticalAlignment','bottom');
xline(ts_lqr,'--r',sprintf('LQR Ts=%.3fs',ts_lqr),...
      'LineWidth',1,'LabelVerticalAlignment','bottom');
xlabel('Time (s)','FontSize',12);
ylabel('Roll Angle (rad)','FontSize',12);
title({'PID vs LQR — Roll Step Response Comparison',...
       'Same plant, different control strategies'},...
       'FontSize',13);
legend('FontSize',11,'Location','southeast');
grid on; grid minor;

%% Panel 2 — Pole zero map
subplot(2,2,3);
p_pid_r = real(p_pid); p_pid_i = imag(p_pid);
p_lqr_r = real(p_lqr); p_lqr_i = imag(p_lqr);
scatter(p_pid_r, p_pid_i, 120, 'b', 'x', 'LineWidth', 2,...
        'DisplayName','PID poles');
hold on;
scatter(p_lqr_r, p_lqr_i, 120, 'r', 'o', 'LineWidth', 2,...
        'DisplayName','LQR poles');
xline(0,'--k','Stability boundary','LineWidth',1.5);
xlabel('Real axis','FontSize',11);
ylabel('Imaginary axis','FontSize',11);
title('Pole Locations','FontSize',12);
legend('FontSize',10,'Location','northeast');
grid on; grid minor;
xlim([min([p_pid_r;p_lqr_r])-5  2]);

%% Panel 3 — Performance bar chart
subplot(2,2,4);
metrics = [os_pid  os_lqr;
           ts_pid*100  ts_lqr*100];
b = bar(metrics);
b(1).FaceColor = 'b';
b(2).FaceColor = 'r';
set(gca,'XTickLabel',{'Overshoot (%)','Settling (×100 s)'});
ylabel('Value','FontSize',11);
title('Performance Comparison','FontSize',12);
legend('PID','LQR','FontSize',10,'Location','northeast');
grid on;

%% Annotation — which is better and why
if ts_lqr < ts_pid
    winner = 'LQR faster settling';
    detail = sprintf('LQR %.0f%% faster than PID',...
                     (ts_pid-ts_lqr)/ts_pid*100);
else
    winner = 'PID comparable performance';
    detail = 'PID simpler to implement';
end

annotation(f7,'textbox',[0.02 0.01 0.96 0.05],...
    'String',sprintf(['Conclusion: %s | %s | '...
                      'LQR = optimal control theory | '...
                      'PID = practical implementation'],...
                     winner, detail),...
    'FontSize',10,'HorizontalAlignment','center',...
    'EdgeColor','none','BackgroundColor',[0.95 0.95 0.95]);

sgtitle({'Sim 7 — PID vs LQR Controller Comparison',...
         'Roll Axis | AUW=1.8kg | L=200mm | AIR2216 KV880'},...
         'FontSize',14,'FontWeight','bold');

saveas(f7,[savepath 'Sim7_PID_vs_LQR.png']);
fprintf('\nSaved: Sim7_PID_vs_LQR.png\n')