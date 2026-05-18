drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

throttle_pct = [50,  55,  60,  65,  75,  85,  100];
thrust_g     = [435, 527, 608, 702, 888, 1076, 1293];
current_A    = [3.5, 4.6, 5.6, 6.8, 9.5, 12.3, 16.2];
power_W      = [56,  73.6,89.6,108.8,152,196.8,259.2];
rpm_data     = [6015,6620,7113,7563,8545,9442,10464];
efficiency   = [7.77,7.16,6.79,6.45,5.84,5.47,4.99];

hover_thrust_per_motor = (m*9.81/4)/9.81*1000;
hover_throttle = interp1(thrust_g,throttle_pct,hover_thrust_per_motor,'linear');
hover_current  = interp1(thrust_g,current_A,hover_thrust_per_motor,'linear');
hover_power    = interp1(thrust_g,power_W,hover_thrust_per_motor,'linear');
hover_rpm      = interp1(thrust_g,rpm_data,hover_thrust_per_motor,'linear');
hover_eff      = interp1(thrust_g,efficiency,hover_thrust_per_motor,'linear');

fprintf('\n+-------- SIM 10 RESULTS --------+\n')
fprintf('| Motor: T-Motor AIR2216 KV880   |\n')
fprintf('| Prop:  T1045 | Battery: 4S 16V |\n')
fprintf('+--------------------------------+\n')
fprintf('| Hover thrust/motor: %.0fg      |\n', hover_thrust_per_motor)
fprintf('| Hover throttle:     %.0f%%      |\n', hover_throttle)
fprintf('| Hover current:      %.1fA      |\n', hover_current)
fprintf('| Hover power/motor:  %.1fW      |\n', hover_power)
fprintf('| Hover RPM:          %.0f       |\n', hover_rpm)
fprintf('| Hover efficiency:   %.2fg/W    |\n', hover_eff)
fprintf('| Total hover power:  %.0fW      |\n', 4*hover_power)
fprintf('| Max thrust (4x):    %.0fg      |\n', 4*1293)
fprintf('| T/W ratio:          %.2f:1     |\n', 4*1293/(m*1000))
fprintf('+--------------------------------+\n')

t_smooth = linspace(50,100,200);
p_thrust = polyfit(throttle_pct,thrust_g,2);
p_power  = polyfit(throttle_pct,power_W,2);
p_eff    = polyfit(throttle_pct,efficiency,2);

thrust_smooth = polyval(p_thrust,t_smooth);
power_smooth  = polyval(p_power,t_smooth);
eff_smooth    = polyval(p_eff,t_smooth);

f10=figure('Name','Sim 10 Thrust Curve',...
           'Color','white','Position',[50 50 1200 850]);

%% Panel 1 — Thrust vs Throttle
subplot(2,2,1);
plot(t_smooth, thrust_smooth,'b','LineWidth',2.5,...
     'DisplayName','Fitted curve'); hold on;
scatter(throttle_pct,thrust_g,100,'r','filled',...
        'DisplayName','Datasheet points');
scatter(hover_throttle,hover_thrust_per_motor,200,'g','filled','p',...
        'DisplayName',sprintf('Hover (%.0f%%,%.0fg)',...
        hover_throttle,hover_thrust_per_motor));
xline(hover_throttle,'--g','LineWidth',1.5);
yline(hover_thrust_per_motor,'--g','LineWidth',1.5);
xlabel('Throttle (%)','FontSize',11);
ylabel('Thrust per Motor (g)','FontSize',11);
title('Thrust vs Throttle','FontSize',12);
legend('FontSize',9,'Location','northwest');
text(hover_throttle+1,hover_thrust_per_motor+30,...
     sprintf('Hover:\n%.0f%% | %.0fg',...
     hover_throttle,hover_thrust_per_motor),...
     'FontSize',10,'Color',[0 0.5 0],'FontWeight','bold');
grid on; grid minor;

%% Panel 2 — Power vs Throttle
subplot(2,2,2);
plot(t_smooth,power_smooth,'r','LineWidth',2.5); hold on;
scatter(throttle_pct,power_W,100,'r','filled');
scatter(hover_throttle,hover_power,200,'g','filled','p');
xline(hover_throttle,'--g','LineWidth',1.5);
yline(hover_power,'--g','LineWidth',1.5);
xlabel('Throttle (%)','FontSize',11);
ylabel('Power per Motor (W)','FontSize',11);
title('Power vs Throttle','FontSize',12);
text(hover_throttle+1,hover_power+5,...
     sprintf('Hover:\n%.0f%% | %.0fW',...
     hover_throttle,hover_power),...
     'FontSize',10,'Color',[0 0.5 0],'FontWeight','bold');
grid on; grid minor;

%% Panel 3 — Efficiency vs Throttle
%% FIXED: use 'g' string not RGB array in plot()
subplot(2,2,3);
plot(t_smooth,eff_smooth,'g','LineWidth',2.5); hold on;
scatter(throttle_pct,efficiency,100,'g','filled');
scatter(hover_throttle,hover_eff,200,'g','filled','p');
xline(hover_throttle,'--g','LineWidth',1.5);
[~,max_idx] = max(eff_smooth);
xline(t_smooth(max_idx),'--r',...
      sprintf('Best eff %.0f%%',t_smooth(max_idx)),...
      'LineWidth',1.5,'LabelVerticalAlignment','bottom');
xlabel('Throttle (%)','FontSize',11);
ylabel('Efficiency (g/W)','FontSize',11);
title('Motor Efficiency vs Throttle','FontSize',12);
text(hover_throttle+1,hover_eff+0.1,...
     sprintf('Hover:\n%.2fg/W',hover_eff),...
     'FontSize',10,'Color',[0 0.5 0],'FontWeight','bold');
grid on; grid minor;

%% Panel 4 — Total thrust vs AUW
subplot(2,2,4);
total_thrust  = thrust_g*4;
total_smooth4 = thrust_smooth*4;
plot(t_smooth,total_smooth4,'b','LineWidth',2.5,...
     'DisplayName','Total thrust 4 motors'); hold on;
scatter(throttle_pct,total_thrust,100,'b','filled');
yline(m*1000,'--r',...
      sprintf('AUW = %.0fg',m*1000),...
      'LineWidth',2,'LabelHorizontalAlignment','left');
yline(m*1000*1.5,':r','1.5x safety margin','LineWidth',1.5);
scatter(hover_throttle,hover_thrust_per_motor*4,...
        200,'g','filled','p',...
        'DisplayName',sprintf('Hover %.0f%%',hover_throttle));
xline(hover_throttle,'--g','LineWidth',1.5);
xlabel('Throttle (%)','FontSize',11);
ylabel('Total Thrust 4 Motors (g)','FontSize',11);
title(sprintf('Total Thrust vs AUW | T/W=%.2f:1 at 100%%',...
      4*1293/(m*1000)),'FontSize',12);
legend('FontSize',9,'Location','northwest');
grid on; grid minor;

sgtitle({'Sim 10 — Motor Thrust Curve Analysis',...
         'T-Motor AIR2216 KV880 + T1045 | 4S 16V | AUW=1.8kg'},...
         'FontSize',14,'FontWeight','bold');

saveas(f10,[savepath 'Sim10_ThrustCurve.png']);
fprintf('\nSaved: Sim10_ThrustCurve.png\n')
fprintf('\nAll 10 simulations COMPLETE!\n')