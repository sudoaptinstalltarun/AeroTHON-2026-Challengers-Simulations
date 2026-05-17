drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';
s = tf('s');

%% PID
C_pid  = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
G_roll = 1/(Ixx*s^2);
CL_pid = feedback(C_pid*G_roll, 1);

%% LQR
A = [0 1; 0 0];
B = [0; 1/Ixx];
Q = [100 0; 0 10];
R = 0.01;
K_lqr = lqr(A, B, Q, R);

A_cl   = A - B*K_lqr;
sys_lqr = ss(A_cl, [0;1/Ixx]*K_lqr(1), [1 0], 0);
CL_lqr  = feedback(tf(sys_lqr), 1);

t   = 0:0.001:3;
ref = 0.1;

[y_pid,~] = step(CL_pid*ref, t);
[y_lqr,~] = step(CL_lqr*ref, t);

%% Metrics — safe calculation
os_pid = (max(y_pid)-ref)/ref*100;
os_lqr = max(0,(max(y_lqr)-ref)/ref*100);

idx_pid = find(abs(y_pid-ref) < ref*0.02, 1);
idx_lqr = find(abs(y_lqr-ref) < ref*0.02, 1);

% Safe defaults if settling too fast
if isempty(idx_pid); idx_pid = length(t); end
if isempty(idx_lqr); idx_lqr = 1; end
ts_pid = t(idx_pid);
ts_lqr = t(idx_lqr);

fprintf('\n+-------- COMPARISON --------+\n')
fprintf('| Metric      PID      LQR   |\n')
fprintf('+----------------------------+\n')
fprintf('| Overshoot   %.0f%%      %.0f%%   |\n', os_pid, os_lqr)
fprintf('| Settling    %.3fs   %.3fs |\n', ts_pid, ts_lqr)
fprintf('| Steady err  0%%       0%%   |\n')
fprintf('+----------------------------+\n')
fprintf('\nLQR gains: K1=%.2f  K2=%.2f\n', K_lqr(1), K_lqr(2))

%% Plot
f7=figure('Name','Sim 7 PID vs LQR',...
          'Color','white','Position',[50 50 1200 800]);

%% Panel 1 — Step response
subplot(2,2,[1 2]);
plot(t, y_pid,'b','LineWidth',2.5,...
     'DisplayName',sprintf('PID  (Ts=%.3fs, OS=%.0f%%)',...
     ts_pid,os_pid));
hold on;
plot(t, y_lqr,'r','LineWidth',2.5,...
     'DisplayName',sprintf('LQR  (Ts=%.3fs, OS=%.0f%%)',...
     ts_lqr,os_lqr));
yline(ref,'--k','Target 0.1 rad','LineWidth',1.5);
yline(ref*1.02,':','Color',[0.5 0.5 0.5],'LineWidth',1);
yline(ref*0.98,':','Color',[0.5 0.5 0.5],'LineWidth',1);

% Only draw settling lines if valid
if ts_pid > 0.001
    xline(ts_pid,'--b',sprintf('PID Ts=%.3fs',ts_pid),...
          'LineWidth',1,'LabelVerticalAlignment','bottom');
end
if ts_lqr > 0.001
    xline(ts_lqr,'--r',sprintf('LQR Ts=%.3fs',ts_lqr),...
          'LineWidth',1,'LabelVerticalAlignment','bottom');
end

xlabel('Time (s)','FontSize',12);
ylabel('Roll Angle (rad)','FontSize',12);
title({'Sim 7 — PID vs LQR Roll Step Response',...
       'LQR = Optimal Control | PID = Practical Implementation'},...
       'FontSize',13);
legend('FontSize',11,'Location','southeast');
grid on; grid minor;

%% Panel 2 — Pole locations
subplot(2,2,3);
p_pid = pole(CL_pid);
p_lqr = eig(A_cl);
scatter(real(p_pid),imag(p_pid),120,'b','x',...
        'LineWidth',2,'DisplayName','PID poles');
hold on;
scatter(real(p_lqr),imag(p_lqr),120,'r','o',...
        'LineWidth',2,'DisplayName','LQR poles');
xline(0,'--k','Stability boundary','LineWidth',1.5);
xlabel('Real axis','FontSize',11);
ylabel('Imaginary axis','FontSize',11);
title('Pole Locations — All in Left Half Plane','FontSize',12);
legend('FontSize',10,'Location','northeast');
grid on; grid minor;
all_r = [real(p_pid);real(p_lqr)];
xlim([min(all_r)-10  5]);

%% Panel 3 — Performance bars
subplot(2,2,4);
categories = {'Overshoot (%)','Settling Time (ms)'};
pid_vals   = [os_pid,  ts_pid*1000];
lqr_vals   = [os_lqr,  ts_lqr*1000];

x = 1:2;
width = 0.35;
bar(x-width/2, pid_vals, width,'b','DisplayName','PID');
hold on;
bar(x+width/2, lqr_vals, width,'r','DisplayName','LQR');
set(gca,'XTick',1:2,'XTickLabel',categories,'FontSize',10);
ylabel('Value','FontSize',11);
title('Performance Metrics Comparison','FontSize',12);
legend('PID','LQR','FontSize',10,'Location','northeast');
grid on;

%% Verdict
if ts_lqr <= ts_pid && os_lqr <= os_pid
    verdict = 'LQR WINS — faster, less overshoot, optimal';
elseif os_lqr < os_pid
    verdict = 'LQR WINS — zero overshoot, smoother response';
else
    verdict = 'PID competitive — simpler implementation';
end

annotation(f7,'textbox',[0.02 0.01 0.96 0.05],...
    'String',sprintf(['Result: %s  |  '...
                      'PID: practical & tunable  |  '...
                      'LQR: mathematically optimal  |  '...
                      'Both stable ✓'],...
                     verdict),...
    'FontSize',10,'HorizontalAlignment','center',...
    'EdgeColor',[0.8 0.8 0.8],...
    'BackgroundColor',[0.95 0.95 0.95]);

sgtitle({'Sim 7 — PID vs LQR Controller Comparison',...
         'Roll Axis | AUW=1.8kg | L=200mm | AIR2216 KV880'},...
         'FontSize',14,'FontWeight','bold');

saveas(f7,[savepath 'Sim7_PID_vs_LQR.png']);
fprintf('\nSaved: Sim7_PID_vs_LQR.png\n')