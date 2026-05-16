%% sim4_battery.m
%% Simulation 4 — Battery & Endurance Analysis
drone_params;

%% Mission power profile
t_mission = [0,   60,  120, 600, 840, 900];
P_mission = [600, 480, 480, 480, 420, 350];

t = 0:1:900;
P = interp1(t_mission, P_mission, t, 'linear');

E_consumed = cumtrapz(t, P);
SOC = 100 * (1 - E_consumed/E_usable);

%% Find RTL trigger time
rtl_idx = find(SOC <= 20, 1);
if ~isempty(rtl_idx)
    rtl_time = t(rtl_idx)/60;
else
    rtl_time = 15;
end

fprintf('\n+------ SIM 4 RESULTS ------+\n')
fprintf('| Total energy:   %.0f J   |\n', E_usable)
fprintf('| Energy used:    %.0f J   |\n', E_consumed(end))
fprintf('| SOC at 15min:   %.0f%%    |\n', SOC(end))
fprintf('| RTL triggers:   %.1fmin  |\n', rtl_time)
fprintf('| Mission OK:     %s        |\n', ...
        string(SOC(end)>20)*"YES ✓" + ...
        string(SOC(end)<=20)*"NO ✗")
fprintf('+----------------------------+\n')

%% Plot
figure('Name','Sim 4 Battery Endurance',...
       'Color','white','Position',[100 100 1000 700]);

subplot(3,1,1);
area(t/60, P, 'FaceColor',[0.2 0.5 0.8],'FaceAlpha',0.7);
hold on;
xregion(0,  1,  'FaceColor',[1.0 0.8 0.8],'FaceAlpha',0.5);
xregion(1,  10, 'FaceColor',[0.8 0.9 1.0],'FaceAlpha',0.5);
xregion(10, 14, 'FaceColor',[0.8 1.0 0.8],'FaceAlpha',0.5);
xregion(14, 15, 'FaceColor',[1.0 0.8 0.8],'FaceAlpha',0.5);
text(0.4, 560,'Takeoff', 'FontSize',10,'FontWeight','bold');
text(4.0, 460,'Mission', 'FontSize',10,'FontWeight','bold');
text(11,  440,'Return',  'FontSize',10,'FontWeight','bold');
text(14.2,380,'Land',    'FontSize',10,'FontWeight','bold');
xlabel('Time (min)','FontSize',11);
ylabel('Power (W)','FontSize',11);
title('Mission Power Profile','FontSize',12);
grid on;

subplot(3,1,2);
plot(t/60, SOC,'r','LineWidth',2.5); hold on;
yline(20,'--k','RTL at 20%','LineWidth',2,...
      'LabelHorizontalAlignment','left');
yline(100,':g','','LineWidth',1);
patch([0 15 15 0],[0 0 20 20],...
      'r','FaceAlpha',0.08,'EdgeColor','none');
text(0.5,10,'RTL Zone','FontSize',10,'Color','r');
text(13, SOC(end)+5,...
     sprintf('End: %.0f%%',SOC(end)),...
     'FontSize',12,'FontWeight','bold','Color','r');
xlabel('Time (min)','FontSize',11);
ylabel('Battery SOC (%)','FontSize',11);
title('Battery State of Charge — 15 min Mission','FontSize',12);
ylim([-5 110]); grid on;

subplot(3,1,3);
E_rem = (E_usable - E_consumed)/1000;
plot(t/60, E_rem,'Color',[0 0.6 0],'LineWidth',2.5); hold on;
yline(0,'--k','Empty','LineWidth',1.5);
xlabel('Time (min)','FontSize',11);
ylabel('Energy Remaining (kJ)','FontSize',11);
title('Usable Energy Remaining','FontSize',12);
grid on;
text(13, E_rem(end)+2,...
     sprintf('%.1f kJ left',E_rem(end)),...
     'FontSize',11,'FontWeight','bold','Color',[0 0.6 0]);

sgtitle({'Sim 4 — Battery & Endurance Analysis',...
         '4S 3300mAh | AUW=1.8kg | 15-min Mission'},...
         'FontSize',14,'FontWeight','bold');