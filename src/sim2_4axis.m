%% sim2_4axis.m
%% Simulation 2 — Full 4-Axis Hover Stability
drone_params;
s = tf('s');

G = {1/(Ixx*s^2), 1/(Iyy*s^2), 1/(Izz*s^2), 1/(m*s^2)};
C = {Kp_rp+Ki_rp/s+Kd_rp*s/(s/N_rp+1), ...
     Kp_rp+Ki_rp/s+Kd_rp*s/(s/N_rp+1), ...
     Kp_y +Ki_y/s +Kd_y*s/(s/N_y+1),   ...
     Kp_z +Ki_z/s +Kd_z*s/(s/N_z+1)};

names  = {'Roll','Pitch','Yaw','Altitude'};
colors = {'b','r',[0 0.6 0],'m'};
refs   = [0.1, 0.1, 0.1, 1.0];
units  = {'rad','rad','rad','m'};

figure('Name','Sim 2 - 4-Axis Stability','Color','white',...
       'Position',[50 50 1100 750]);

fprintf('\n+-------- SIM 2 RESULTS --------+\n')
fprintf('| Axis       Status    Ts       |\n')
fprintf('+--------------------------------+\n')

for i = 1:4
    CL = feedback(C{i}*G{i}, 1);
    [y,t] = step(CL*refs(i), 0:0.001:6);

    subplot(2,2,i);
    plot(t, y, 'Color',colors{i}, 'LineWidth',2.5);
    hold on;
    yline(refs(i),'--k','Target','LineWidth',1.5);
    xlabel('Time (s)','FontSize',11);
    ylabel([names{i} ' (' units{i} ')'],'FontSize',11);
    title([names{i} ' Step Response'],'FontSize',12);
    grid on; grid minor;

    % Stability check
    p  = pole(CL);
    ok = all(real(p) < 0);
    clr = [0 0.5 0]*ok + [1 0 0]*(~ok);
    if ok; txt='STABLE ✓'; else; txt='UNSTABLE ✗'; end
    text(3.5, refs(i)*0.25, txt, 'FontSize',12,...
         'FontWeight','bold','Color',clr);

    % Settling time
    idx = find(abs(y-refs(i)) < refs(i)*0.02, 1);
    if ~isempty(idx)
        Ts = t(idx);
        xline(Ts,'--','Color',[0.5 0.5 0.5],...
              'Label',sprintf('Ts=%.2fs',Ts),...
              'LabelVerticalAlignment','bottom');
        fprintf('| %-10s %-9s %.3fs  |\n', names{i}, txt, Ts)
    else
        fprintf('| %-10s %-9s ---      |\n', names{i}, txt)
    end
end
fprintf('+--------------------------------+\n')

sgtitle({'Sim 2 - Full 4-Axis Hover Stability',...
         'X-Frame | AUW=1.8kg | L=200mm | AIR2216 KV880'},...
         'FontSize',14,'FontWeight','bold');