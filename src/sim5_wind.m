%% sim5_wind.m
%% Simulation 5 — Wind Disturbance Rejection
drone_params;
s = tf('s');

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

%% Roll plant and controller
G_roll = 1/(Ixx*s^2);
C_roll = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
CL     = feedback(C_roll*G_roll, 1);

%% Disturbance path — wind enters at plant input
G_dist = feedback(G_roll, C_roll);

t = 0:0.001:8;

%% Reference response — no wind
[y_ref, ~] = step(CL*0.1, t);

%% Wind gust — 3 different speeds
wind_speeds = [3, 5, 8];   % m/s
wind_time   = 3.0;
wind_dur    = 0.5;
colors_w    = {[0 0.6 1], [1 0.5 0], [1 0 0]};

figure('Name','Sim 5 Wind Disturbance',...
       'Color','white','Position',[100 100 1100 750]);

%% Panel 1 — All wind speeds on one plot
subplot(2,2,[1 2]);
plot(t, y_ref,'--k','LineWidth',1.5,...
     'DisplayName','No wind'); hold on;

fprintf('\n+------ SIM 5 RESULTS ------+\n')
fprintf('| Wind    MaxDev   Recovery |\n')
fprintf('+----------------------------+\n')

for wi = 1:3
    ws = wind_speeds(wi);
    dist_amp = ws * 0.03;

    dist = zeros(size(t));
    dist(t >= wind_time & t <= wind_time+wind_dur) = dist_amp;

    [y_gust,~] = lsim(G_dist, dist, t);
    y_total    = y_ref + y_gust;

    plot(t, y_total,'Color',colors_w{wi},...
         'LineWidth',2,...
         'DisplayName',sprintf('Wind %.0fm/s',ws));

    % Recovery time
    after_gust = t > wind_time + wind_dur;
    y_after    = y_total(after_gust);
    t_after    = t(after_gust);
    rec_idx    = find(abs(y_after - 0.1) < 0.005, 1);
    if ~isempty(rec_idx)
        rec_t = t_after(rec_idx) - wind_time;
    else
        rec_t = 999;
    end

    max_dev = max(abs(y_total - 0.1));
    fprintf('| %4.0fm/s  %.4frad  %.2fs   |\n',...
            ws, max_dev, rec_t)
end
fprintf('+----------------------------+\n')

% Gust zone
xregion(wind_time, wind_time+wind_dur,...
        'FaceColor',[1 0.9 0],'FaceAlpha',0.3);
text(wind_time+0.05, 0.145,...
     sprintf('Wind gust\n%.1fs duration',wind_dur),...
     'FontSize',10,'Color',[0.6 0.4 0]);
yline(0.1,'--','Color',[0.5 0.5 0.5],'LineWidth',1);
xlabel('Time (s)','FontSize',12);
ylabel('Roll Angle (rad)','FontSize',12);
title({'Wind Disturbance Rejection — Roll Axis',...
       'PID Controller recovers from wind gusts'},...
       'FontSize',13);
legend('FontSize',10,'Location','southeast');
grid on; grid minor;

%% Panel 2 — 5m/s detailed with recovery markers
subplot(2,2,3);
ws = 5;
dist_amp = ws * 0.03;
dist = zeros(size(t));
dist(t >= wind_time & t <= wind_time+wind_dur) = dist_amp;
[y_gust,~] = lsim(G_dist, dist, t);
y_total = y_ref + y_gust;

plot(t, y_ref,'--k','LineWidth',1.5); hold on;
plot(t, y_total,'r','LineWidth',2.5);
xregion(wind_time,wind_time+wind_dur,...
        'FaceColor',[1 0.9 0],'FaceAlpha',0.4);

after_gust = t > wind_time+wind_dur;
y_after = y_total(after_gust);
t_after = t(after_gust);
rec_idx = find(abs(y_after-0.1) < 0.005,1);
if ~isempty(rec_idx)
    xline(t_after(rec_idx),'--g',...
          sprintf('Recovered %.2fs',t_after(rec_idx)-wind_time),...
          'LineWidth',1.5,'FontSize',10,...
          'LabelVerticalAlignment','bottom');
end

yline(0.1,'--','Color',[0.5 0.5 0.5]);
xlabel('Time (s)','FontSize',11);
ylabel('Roll Angle (rad)','FontSize',11);
title('5 m/s Gust — Detailed View','FontSize',12);
legend('No wind','With 5m/s gust','FontSize',9,...
       'Location','southeast');
grid on; grid minor;

%% Panel 3 — Disturbance signal
subplot(2,2,4);
dist_show = zeros(size(t));
dist_show(t>=wind_time & t<=wind_time+wind_dur) = 5;
area(t, dist_show,'FaceColor',[1 0.8 0],'FaceAlpha',0.7);
hold on;
yline(0,'--k');
xlabel('Time (s)','FontSize',11);
ylabel('Wind Speed (m/s)','FontSize',11);
title('Wind Gust Profile','FontSize',12);
ylim([-1 10]);
text(wind_time+0.05, 6,...
     sprintf('%.1fs gust\n@ t=%.0fs',wind_dur,wind_time),...
     'FontSize',10,'FontWeight','bold');
grid on; grid minor;

sgtitle({'Sim 5 — Wind Disturbance Rejection',...
         'AUW=1.8kg | L=200mm | PID Controller'},...
         'FontSize',14,'FontWeight','bold');

f5 = gcf;
saveas(f5,[savepath 'Sim5_Wind.png']);
fprintf('\nSaved: Sim5_Wind.png\n')