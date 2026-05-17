%% sim9_payload_cg.m
%% Simulation 9 — Payload Drop CG Shift Analysis
drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';
s = tf('s');

%% Component positions (mm from frame centre, Z=0 at bottom plate)
components = [
    400,   0,    0,   44;   % frame
     65, 141, -141,   88;   % motor 1 FR (200mm arm at 45deg)
     65, 141,  141,   88;   % motor 2 FL
     65,-141,  141,   88;   % motor 3 RL
     65,-141, -141,   88;   % motor 4 RR
     68,   0,    0,   91;   % props
     84,   0,    0,   50;   % ESCs
    220,   0,    0,   20;   % battery
     38,   0,    0,   70;   % Pixhawk
    136,   0,   20,   60;   % Jetson
     55,  30,    0,   35;   % T265
      3,   0,    0,   88;   % camera
     17,  40,    0,   30;   % lidars
     50,   0,    0,   44;   % wiring
];

%% Payload position (attached below centre hub)
payload_mass = 100;   % grams
payload_z    = 0;     % mm — hanging below frame

%% CG with payload
all_loaded = [components; payload_mass, 0, 0, payload_z];
m_comp  = all_loaded(:,1)/1000;
x_comp  = all_loaded(:,2)/1000;
y_comp  = all_loaded(:,3)/1000;
z_comp  = all_loaded(:,4)/1000;

total_loaded = sum(m_comp);
CGx_loaded = sum(m_comp.*x_comp)/total_loaded*1000;
CGy_loaded = sum(m_comp.*y_comp)/total_loaded*1000;
CGz_loaded = sum(m_comp.*z_comp)/total_loaded*1000;

%% CG without payload (after drop)
m_comp2  = components(:,1)/1000;
x_comp2  = components(:,2)/1000;
y_comp2  = components(:,3)/1000;
z_comp2  = components(:,4)/1000;

total_empty = sum(m_comp2);
CGx_empty = sum(m_comp2.*x_comp2)/total_empty*1000;
CGy_empty = sum(m_comp2.*y_comp2)/total_empty*1000;
CGz_empty = sum(m_comp2.*z_comp2)/total_empty*1000;

%% CG shift
dCGx = CGx_empty - CGx_loaded;
dCGy = CGy_empty - CGy_loaded;
dCGz = CGz_empty - CGz_loaded;

fprintf('\n+-------- SIM 9 RESULTS — CG ANALYSIS --------+\n')
fprintf('| State        CGx(mm)  CGy(mm)  CGz(mm)      |\n')
fprintf('+----------------------------------------------+\n')
fprintf('| With payload  %6.2f   %6.2f   %6.2f       |\n',...
        CGx_loaded, CGy_loaded, CGz_loaded)
fprintf('| After drop    %6.2f   %6.2f   %6.2f       |\n',...
        CGx_empty, CGy_empty, CGz_empty)
fprintf('| CG shift      %6.2f   %6.2f   %6.2f mm    |\n',...
        dCGx, dCGy, dCGz)
fprintf('+----------------------------------------------+\n')
fprintf('| Ideal: CGx=0, CGy=0 (perfectly centred)      |\n')
if abs(CGx_empty)<5 && abs(CGy_empty)<5
    fprintf('| Status: CG within 5mm — ACCEPTABLE ✓         |\n')
else
    fprintf('| Status: CG offset > 5mm — ADJUST LAYOUT      |\n')
end
fprintf('+----------------------------------------------+\n')

%% Altitude stability before and after drop
G_loaded  = 1/(total_loaded*s^2);
G_unloaded= 1/(total_empty*s^2);
C_alt     = Kp_z + Ki_z/s + Kd_z*s/(s/N_z+1);

CL_loaded  = feedback(C_alt*G_loaded,  1);
CL_unloaded= feedback(C_alt*G_unloaded,1);

t_before = 0:0.001:7;
t_after  = 0:0.001:8;
[y1,~] = step(CL_loaded*5,   t_before);
[y2,~] = step(CL_unloaded*5, t_after);

%% Plot
f9=figure('Name','Sim 9 Payload CG',...
          'Color','white','Position',[50 50 1200 850]);

%% Panel 1 — CG position bar chart
subplot(2,2,1);
cg_data = [CGx_loaded CGx_empty;
           CGy_loaded CGy_empty;
           CGz_loaded CGz_empty];
b=bar(cg_data);
b(1).FaceColor=[0.2 0.5 0.8];
b(2).FaceColor=[0.9 0.4 0.2];
set(gca,'XTickLabel',{'CGx (Forward)','CGy (Lateral)','CGz (Vertical)'});
ylabel('CG Position (mm)','FontSize',11);
title('Centre of Gravity — Before vs After Drop','FontSize',12);
legend('With payload','After drop','FontSize',10,...
       'Location','northeast');
yline(0,'--k','Ideal centre','LineWidth',1.5);
grid on;

%% Panel 2 — CG shift arrows (top view)
subplot(2,2,2);
scatter(CGx_loaded, CGy_loaded, 200,'b','filled',...
        'DisplayName','CG with payload');
hold on;
scatter(CGx_empty, CGy_empty, 200,'r','filled',...
        'DisplayName','CG after drop');
quiver(CGx_loaded, CGy_loaded,...
       dCGx, dCGy,...
       0,'k','LineWidth',2,'MaxHeadSize',2);
scatter(0, 0, 100,'k','x','LineWidth',2,...
        'DisplayName','Geometric centre');

% Draw drone frame outline
theta_arm = [45 135 225 315]*pi/180;
for a=1:4
    plot([0 200*cos(theta_arm(a))],[0 200*sin(theta_arm(a))],...
         'Color',[0.6 0.6 0.6],'LineWidth',2);
    scatter(200*cos(theta_arm(a)),200*sin(theta_arm(a)),...
            80,'k','filled');
end

xlabel('X — Forward (mm)','FontSize',11);
ylabel('Y — Lateral (mm)','FontSize',11);
title({'CG Shift — Top View',...
       sprintf('Shift: X=%.2fmm Y=%.2fmm',dCGx,dCGy)},...
       'FontSize',12);
legend('FontSize',9,'Location','northeast');
xlim([-250 250]); ylim([-250 250]);
grid on; axis equal;

%% Panel 3 — Altitude response before/after drop
subplot(2,2,[3 4]);
plot(t_before, y1,'b','LineWidth',2,...
     'DisplayName',sprintf('With payload (%.1fkg)',total_loaded));
hold on;

% Continued flight after drop
y2_continued = y1(end) + (y2 - y2(1));
t_continued  = 7 + t_after;
plot(t_continued, y2_continued,'r','LineWidth',2,...
     'DisplayName',sprintf('After drop (%.1fkg)',total_empty));

xline(7,'--k','Payload released','LineWidth',2,...
      'LabelVerticalAlignment','bottom','FontSize',10);
yline(5,'--','Color',[0.5 0.5 0.5],'LineWidth',1);

% CG shift annotation
text(7.2, y2_continued(1)+0.3,...
     sprintf('CG shifts up\n%.2fmm',abs(dCGz)),...
     'FontSize',10,'Color','r','FontWeight','bold');

xlabel('Time (s)','FontSize',12);
ylabel('Altitude (m)','FontSize',12);
title({'Altitude Response — Payload Drop at t=7s',...
       'Drone adjusts to new mass and CG automatically'},...
       'FontSize',12);
legend('FontSize',11,'Location','southeast');
grid on; grid minor;

sgtitle({'Sim 9 — Payload Drop CG Shift Analysis',...
         'AeroTHON 2026 | 100g Payload | AUW=1.8kg | L=200mm'},...
         'FontSize',14,'FontWeight','bold');

saveas(f9,[savepath 'Sim9_Payload_CG.png']);
fprintf('\nSaved: Sim9_Payload_CG.png\n')