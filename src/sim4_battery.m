drone_params;

%% Realistic power for 1.8kg — mixed mission (not all hover)
%% Hover = 180W, Forward flight = 120W (more efficient)
t_mission = [0,   30,  60,  600, 780, 840, 870, 900];
P_mission = [200, 180, 150, 150, 150, 160, 100,  80];
%            TKO  HOV  FWD  FWD  RET  HOV  DSC  LND

t = 0:1:900;
P = interp1(t_mission, P_mission, t, 'linear');

E_consumed = cumtrapz(t, P);
SOC = 100 * (1 - E_consumed/E_usable);

rtl_idx = find(SOC <= 20, 1);
if ~isempty(rtl_idx)
    rtl_time = t(rtl_idx)/60;
    mission_ok = 'MARGINAL';
else
    rtl_time = 15.0;
    mission_ok = 'YES';
end

fprintf('\n+------ SIM 4 RESULTS ------+\n')
fprintf('| Total energy:  %.0f J    |\n', E_usable)
fprintf('| Energy used:   %.0f J    |\n', E_consumed(end))
fprintf('| SOC at 15min:  %.0f%%     |\n', SOC(end))
fprintf('| RTL at:        %.1f min  |\n', rtl_time)
fprintf('| Mission OK:    %s        |\n', mission_ok)
fprintf('+----------------------------+\n')

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

f4=figure('Name','Sim 4 Battery',...
          'Color','white','Position',[100 100 1000 700]);

subplot(3,1,1);
area(t/60, P,'FaceColor',[0.2 0.5 0.8],'FaceAlpha',0.7);
hold on;
xregion(0,  0.5,'FaceColor',[1.0 0.8 0.8],'FaceAlpha',0.5);
xregion(0.5,1,  'FaceColor',[0.9 0.9 0.5],'FaceAlpha',0.5);
xregion(1,  10, 'FaceColor',[0.8 0.9 1.0],'FaceAlpha',0.5);
xregion(10, 14, 'FaceColor',[0.8 1.0 0.8],'FaceAlpha',0.5);
xregion(14, 15, 'FaceColor',[1.0 0.8 0.8],'FaceAlpha',0.5);
text(0.1, 190,'TKO', 'FontSize',9,'FontWeight','bold');
text(0.6, 170,'HOV', 'FontSize',9,'FontWeight','bold');
text(4.0, 140,'Forward Flight','FontSize',9,'FontWeight','bold');
text(11,  145,'Return','FontSize',9,'FontWeight','bold');
text(14.2,90, 'Land', 'FontSize',9,'FontWeight','bold');
xlabel('Time (min)','FontSize',11);
ylabel('Power (W)','FontSize',11);
title('Mission Power Profile — Mixed Flight','FontSize',12);
ylim([0 250]); grid on;

subplot(3,1,2);
plot(t/60, SOC,'r','LineWidth',2.5); hold on;
yline(20,'--k','RTL at 20%','LineWidth',2,...
      'LabelHorizontalAlignment','left');
yline(0,'--','Color',[0.5 0.5 0.5],'LineWidth',1);
patch([0 15 15 0],[0 0 20 20],...
      'r','FaceAlpha',0.08,'EdgeColor','none');
text(0.5,10,'RTL Zone','FontSize',10,'Color','r');

final_soc = SOC(end);
soc_color = 'r';
if final_soc > 20; soc_color = [0 0.6 0]; end
text(13, final_soc+5,...
     sprintf('End: %.0f%%',final_soc),...
     'FontSize',12,'FontWeight','bold','Color',soc_color);

% RTL marker
if ~isempty(rtl_idx)
    xline(rtl_time,'--b',...
          sprintf('RTL %.1fmin',rtl_time),...
          'LineWidth',1.5,'FontSize',10,...
          'LabelVerticalAlignment','bottom');
end

xlabel('Time (min)','FontSize',11);
ylabel('Battery SOC (%)','FontSize',11);
title('Battery State of Charge — 15 min Mission','FontSize',12);
ylim([-5 110]); grid on;

subplot(3,1,3);
E_rem = (E_usable - E_consumed)/1000;
area(t/60, max(E_rem,0),'FaceColor',[0.2 0.7 0.3],'FaceAlpha',0.6);
hold on;
area(t/60, min(E_rem,0),'FaceColor',[1 0.3 0.3],'FaceAlpha',0.6);
yline(0,'--k','Empty','LineWidth',1.5);
xlabel('Time (min)','FontSize',11);
ylabel('Energy Remaining (kJ)','FontSize',11);
title('Usable Energy Remaining','FontSize',12);
grid on;

% Design recommendation annotation
if final_soc < 20
    annotation(f4,'textbox',[0.55 0.02 0.42 0.06],...
        'String',sprintf(['Design note: 5000mAh battery recommended ' ...
                          '(+%.0fg, +%.0f%%% endurance)'],...
                         100, 50),...
        'FontSize',9,'Color',[0.7 0.3 0],...
        'EdgeColor','none','BackgroundColor','none');
end

sgtitle({'Sim 4 — Battery & Endurance Analysis',...
         '4S 3300mAh | AUW=1.8kg | 15-min Mission'},...
         'FontSize',14,'FontWeight','bold');

saveas(f4,[savepath 'Sim4_Battery.png']);
fprintf('\nSaved: Sim4_Battery.png\n')
fprintf('\nDesign note: Consider 5000mAh battery\n')
fprintf('  Weight penalty: +~100g\n')
fprintf('  Endurance gain: +~50%%\n')
fprintf('  New AUW: ~1.9kg (still under 2kg limit)\n')