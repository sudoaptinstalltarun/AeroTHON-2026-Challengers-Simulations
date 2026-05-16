drone_params;
s = tf('s');

% ── Aggressive fix for Yaw ──────────────────────
% Problem: Izz too small → system oscillates
% Fix: Pure PD for yaw (remove integral completely)
Kp_y_fix = 0.8;
Ki_y_fix = 0.0;   % remove integral
Kd_y_fix = 0.3;
N_y_fix  = 20;

% ── Aggressive fix for Altitude ─────────────────
% Problem: double integrator + Ki = unstable
% Fix: PD only, no integral for altitude
Kp_z_fix = 4.0;
Ki_z_fix = 0.0;   % remove integral
Kd_z_fix = 6.0;
N_z_fix  = 20;

C_roll  = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
C_pitch = Kp_rp + Ki_rp/s + Kd_rp*s/(s/N_rp+1);
C_yaw   = Kp_y_fix + Ki_y_fix/s + Kd_y_fix*s/(s/N_y_fix+1);
C_alt   = Kp_z_fix + Ki_z_fix/s + Kd_z_fix*s/(s/N_z_fix+1);

G_roll  = 1/(Ixx*s^2);
G_pitch = 1/(Iyy*s^2);
G_yaw   = 1/(Izz*s^2);
G_alt   = 1/(m*s^2);

CL_roll  = feedback(C_roll*G_roll,   1);
CL_pitch = feedback(C_pitch*G_pitch, 1);
CL_yaw   = feedback(C_yaw*G_yaw,     1);
CL_alt   = feedback(C_alt*G_alt,     1);

% Check poles first
fprintf('\n+-------- POLE CHECK --------+\n')
sys_all = {CL_roll, CL_pitch, CL_yaw, CL_alt};
names   = {'Roll','Pitch','Yaw','Altitude'};
for i = 1:4
    p  = pole(sys_all{i});
    ok = all(real(p) < 0);
    if ok; st='STABLE ✓'; else; st='UNSTABLE ✗'; end
    fprintf('| %-10s %s\n', names{i}, st)
end
fprintf('+----------------------------+\n')

refs   = [0.1, 0.1, 0.1, 1.0];
colors = {'b','r',[0 0.6 0],'m'};
units  = {'rad','rad','rad','m'};

figure('Name','Sim 2 FIXED','Color','white',...
       'Position',[50 50 1100 750]);

for i = 1:4
    CL = sys_all{i};
    [y,t] = step(CL*refs(i), 0:0.001:10);

    subplot(2,2,i);
    plot(t, y,'Color',colors{i},'LineWidth',2.5); hold on;
    yline(refs(i),'--k','Target','LineWidth',1.5);
    xlabel('Time (s)','FontSize',11);
    ylabel([names{i} ' (' units{i} ')'],'FontSize',11);
    title([names{i} ' — Fixed'],'FontSize',12);
    grid on; grid minor;

    p  = pole(CL);
    ok = all(real(p) < 0);
    clr = [0 0.5 0]*ok + [1 0 0]*(~ok);
    if ok; txt='STABLE ✓'; else; txt='UNSTABLE ✗'; end
    text(5.0,refs(i)*0.3,txt,'FontSize',12,...
         'FontWeight','bold','Color',clr);

    idx = find(abs(y-refs(i)) < refs(i)*0.02, 1);
    if ~isempty(idx)
        xline(t(idx),'--','Color',[0.5 0.5 0.5],...
              'Label',sprintf('Ts=%.2fs',t(idx)),...
              'LabelVerticalAlignment','bottom');
    end
end

sgtitle({'Sim 2 FIXED - Full 4-Axis Hover Stability',...
         'X-Frame | AUW=1.8kg | L=200mm | AIR2216 KV880'},...
         'FontSize',14,'FontWeight','bold');