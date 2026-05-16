drone_params;

savepath = 'C:\Users\tarun\Desktop\AeroTHON-2026-Challengers-Simulations\src\';

t_wp  = [0,5,8,11,25,30,50,55,65,70,85,90,104,107,112,120];
t_sim = 0:0.05:120; dt = 0.05;

WPs = [0,0,0;0,0,5;1,0,5;1,0,3;11,0,3;11,0,10;
       11,15,10;11,15,5;11,15,5;11,15,10;11,0,10;
       11,0,3;1,0,3;1,0,5;0,0,1;0,0,0];

x_ref=interp1(t_wp,WPs(:,1),t_sim,'pchip');
y_ref=interp1(t_wp,WPs(:,2),t_sim,'pchip');
z_ref=interp1(t_wp,WPs(:,3),t_sim,'pchip');

Kp_p=1.2; Kd_p=0.8;
x_a=zeros(size(t_sim)); y_a=x_a; z_a=x_a;
vx=0; vy=0; vz=0;
for i=2:length(t_sim)
    ax=max(-3,min(3,Kp_p*(x_ref(i)-x_a(i-1))-Kd_p*vx));
    ay=max(-3,min(3,Kp_p*(y_ref(i)-y_a(i-1))-Kd_p*vy));
    az=max(-3,min(3,Kp_p*(z_ref(i)-z_a(i-1))-Kd_p*vz));
    vx=vx+ax*dt; vy=vy+ay*dt; vz=vz+az*dt;
    x_a(i)=x_a(i-1)+vx*dt;
    y_a(i)=y_a(i-1)+vy*dt;
    z_a(i)=z_a(i-1)+vz*dt;
end

roll_a  = smoothdata(atan2(gradient(gradient(y_a,dt),dt),g),'gaussian',20);
pitch_a = smoothdata(atan2(-gradient(gradient(x_a,dt),dt),g),'gaussian',20);
yaw_a   = smoothdata(atan2(gradient(y_a,dt),max(gradient(x_a,dt),0.001)),'gaussian',30);

fprintf('\n+------ SIM 3 RESULTS ------+\n')
fprintf('| X RMS error:  %.3f m     |\n', sqrt(mean((x_a-x_ref).^2)))
fprintf('| Y RMS error:  %.3f m     |\n', sqrt(mean((y_a-y_ref).^2)))
fprintf('| Z RMS error:  %.3f m     |\n', sqrt(mean((z_a-z_ref).^2)))
fprintf('| Max altitude: %.1f m     |\n', max(z_a))
fprintf('| Mission time: 120 s      |\n')
fprintf('+----------------------------+\n')

et=[8,25,65,90,104];
en={'QR','Corr Out','Drop','Ret In','RTH'};

%% ── GRAPH 1: 6-panel trajectory ──────────────────
fprintf('\nGenerating Graph 1 — Trajectory...\n')
f1=figure('Name','Sim3 Trajectory',...
          'Color','white','Position',[30 30 1200 800]);

subplot(3,2,1);
plot(t_sim,x_ref,'--k','LineWidth',1.5); hold on;
plot(t_sim,x_a,'b','LineWidth',2);
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6],...
    'Label',en{e},'FontSize',8); end
ylabel('X Forward (m)','FontSize',11);
title('Forward Position','FontSize',12);
legend('Reference','Actual','FontSize',9,'Location','northwest');
grid on; grid minor;

subplot(3,2,2);
plot(t_sim,y_ref,'--k','LineWidth',1.5); hold on;
plot(t_sim,y_a,'r','LineWidth',2);
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6]); end
ylabel('Y Lateral (m)','FontSize',11);
title('Lateral Position','FontSize',12);
legend('Reference','Actual','FontSize',9,'Location','northwest');
grid on; grid minor;

subplot(3,2,3);
plot(t_sim,z_ref,'--k','LineWidth',1.5); hold on;
plot(t_sim,z_a,'m','LineWidth',2);
yline(3,':b','Corridor 3m','FontSize',9);
yline(5,':g','QR/Drop 5m','FontSize',9);
yline(10,':r','Delivery 10m','FontSize',9);
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6]); end
ylabel('Altitude (m)','FontSize',11);
title('Altitude Profile','FontSize',12);
legend('Reference','Actual','FontSize',9,'Location','northwest');
grid on; grid minor;

subplot(3,2,4);
plot(t_sim,rad2deg(roll_a),'b','LineWidth',2); hold on;
yline(0,'--k'); yline(15,':r','Max 15'); yline(-15,':r');
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6]); end
ylabel('Roll (deg)','FontSize',11);
title('Roll Angle','FontSize',12);
grid on; grid minor;

subplot(3,2,5);
plot(t_sim,rad2deg(pitch_a),'r','LineWidth',2); hold on;
yline(0,'--k'); yline(15,':r','Max 15'); yline(-15,':r');
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6]); end
ylabel('Pitch (deg)','FontSize',11);
title('Pitch Angle','FontSize',12);
xlabel('Time (s)','FontSize',10);
grid on; grid minor;

subplot(3,2,6);
plot(t_sim,rad2deg(yaw_a),'g','LineWidth',2); hold on;
yline(0,'--k');
for e=1:5; xline(et(e),'--','Color',[0.6 0.6 0.6]); end
ylabel('Yaw (deg)','FontSize',11);
title('Yaw Angle','FontSize',12);
xlabel('Time (s)','FontSize',10);
grid on; grid minor;

sgtitle({'Sim 3 - Mission 2 Trajectory Tracking',...
         'AeroTHON 2026 | AUW=1.8kg | L=200mm'},...
         'FontSize',14,'FontWeight','bold');

saveas(f1,[savepath 'Sim3_Trajectory_6panel.png']);
fprintf('Saved: Sim3_Trajectory_6panel.png\n')

%% ── GRAPH 2: 3D static path ──────────────────────
fprintf('Generating Graph 2 — 3D Path...\n')
f2=figure('Name','Sim3 3D Path',...
          'Color','white','Position',[100 100 900 700]);

plot3(x_a,y_a,z_a,'b','LineWidth',2.5); hold on;
scatter3(WPs(:,1),WPs(:,2),WPs(:,3),80,'r','filled');
plot3([1 11],[1.75 1.75],[3 3],'r--','LineWidth',2);
plot3([1 11],[-1.75 -1.75],[3 3],'r--','LineWidth',2);
plot3([1 11],[1.75 1.75],[0 0],'r:','LineWidth',1);
plot3([1 11],[-1.75 -1.75],[0 0],'r:','LineWidth',1);
scatter3(0,0,0,200,'g','filled');
scatter3(11,15,5,250,'m','filled','d');

lbl={'Start','QR Scan','Corr In','Corr Out',...
     'Delivery','Drop Point','Land'};
kid=[1,3,4,5,7,8,16];
for i=1:7
    k=kid(i);
    text(WPs(k,1)+0.3,WPs(k,2)+0.5,WPs(k,3)+0.5,...
         lbl{i},'FontSize',9,'FontWeight','bold');
end

text(6,0,2.2,'Corridor 3.5m wide','FontSize',10,...
     'Color','r','HorizontalAlignment','center');
xlabel('X Forward (m)','FontSize',12);
ylabel('Y Lateral (m)','FontSize',12);
zlabel('Altitude (m)','FontSize',12);
title({'Sim 3 - Mission 2: 3D Flight Path',...
       'SkyScan Autonomous Rapid Delivery | AeroTHON 2026'},...
       'FontSize',13,'FontWeight','bold');
grid on; view(45,30);
legend('Flight path','Waypoints','FontSize',10,...
       'Location','northwest');

saveas(f2,[savepath 'Sim3_3D_FlightPath.png']);
fprintf('Saved: Sim3_3D_FlightPath.png\n')

%% ── VIDEO: 3D animation ──────────────────────────
fprintf('\nGenerating Video...\n')
fprintf('DO NOT touch screen until 100%%\n\n')

arm_dirs=[cos(pi/4) sin(pi/4) 0;-cos(pi/4) sin(pi/4) 0;
          -cos(pi/4) -sin(pi/4) 0;cos(pi/4) -sin(pi/4) 0];
th_p=linspace(0,2*pi,25);
skip=6; frames=1:skip:length(t_sim);

fig_v=figure('Name','3D Animation DO NOT CLOSE',...
             'Color','black','Units','pixels',...
             'Position',[50 50 1280 720],...
             'MenuBar','none','ToolBar','none',...
             'CloseRequestFcn','');
drawnow;
frame0=getframe(fig_v);
fsize=size(frame0.cdata);
fw=fsize(2); fh=fsize(1);
fprintf('Frame size: %d x %d\n',fw,fh)

v=VideoWriter([savepath 'aerothon_mission2.mp4'],'MPEG-4');
v.FrameRate=30; v.Quality=95;
open(v);

for fi=1:length(frames)
    i=frames(fi);

    if ~ishandle(fig_v)||~isvalid(fig_v)
        fig_v=figure('Name','3D Animation DO NOT CLOSE',...
                     'Color','black','Units','pixels',...
                     'Position',[50 50 1280 720],...
                     'MenuBar','none','ToolBar','none',...
                     'CloseRequestFcn','');
    end

    clf(fig_v); set(fig_v,'Color','black');
    ax=axes('Parent',fig_v,'Color','black',...
            'XColor','w','YColor','w','ZColor','w',...
            'GridColor',[0.3 0.3 0.3]);
    hold(ax,'on');

    px=x_a(i); py=y_a(i); pz=z_a(i);
    ro=roll_a(i); pi_=pitch_a(i); ya=yaw_a(i);

    Rx=[1 0 0;0 cos(ro) -sin(ro);0 sin(ro) cos(ro)];
    Ry=[cos(pi_) 0 sin(pi_);0 1 0;-sin(pi_) 0 cos(pi_)];
    Rz=[cos(ya) -sin(ya) 0;sin(ya) cos(ya) 0;0 0 1];
    R=Rz*Ry*Rx;

    ts=max(1,i-60);
    plot3(ax,x_a(ts:i),y_a(ts:i),z_a(ts:i),'c','LineWidth',2);
    plot3(ax,x_a,y_a,z_a,'Color',[0.25 0.25 0.25],'LineWidth',0.8);

    for arm=1:4
        ae=(R*arm_dirs(arm,:)')';
        plot3(ax,[px px+L*ae(1)],[py py+L*ae(2)],...
              [pz pz+L*ae(3)],'w','LineWidth',3);
        scatter3(ax,px+L*ae(1),py+L*ae(2),pz+L*ae(3),...
                 100,'y','filled');
        spin=fi*0.8+arm*pi/2;
        pp=prop_r*[cos(th_p+spin);sin(th_p+spin);zeros(size(th_p))];
        pp_r=R*pp;
        fill3(ax,px+L*ae(1)+pp_r(1,:),...
                 py+L*ae(2)+pp_r(2,:),...
                 pz+L*ae(3)+pp_r(3,:),...
              [0.2 0.9 0.2],'FaceAlpha',0.35,'EdgeColor','none');
    end

    scatter3(ax,px,py,pz,180,'r','filled');

    [gx,gy]=meshgrid(-2:4:22,-4:4:20);
    surf(ax,gx,gy,zeros(size(gx)),...
         'FaceColor',[0.08 0.15 0.08],...
         'FaceAlpha',0.4,'EdgeColor',[0.15 0.25 0.15]);

    plot3(ax,[1 11],[1.75 1.75],[3 3],'r--','LineWidth',2);
    plot3(ax,[1 11],[-1.75 -1.75],[3 3],'r--','LineWidth',2);
    plot3(ax,[1 11],[1.75 1.75],[0 0],'r:','LineWidth',1);
    plot3(ax,[1 11],[-1.75 -1.75],[0 0],'r:','LineWidth',1);

    scatter3(ax,WPs(:,1),WPs(:,2),WPs(:,3),30,'y','filled','^');
    scatter3(ax,11,15,0,300,'m','filled','d');
    text(11,15,0.6,'DROP ZONE','Color','m','FontSize',9,...
         'FontWeight','bold','HorizontalAlignment','center','Parent',ax);
    scatter3(ax,0,0,0,200,'g','filled');
    text(0.5,0.5,0.5,'HOME','Color','g','FontSize',9,...
         'FontWeight','bold','Parent',ax);

    xlim(ax,[px-10 px+10]);
    ylim(ax,[py-10 py+10]);
    zlim(ax,[0 14]);
    view(ax,45,28); grid(ax,'on');

    title(ax,sprintf(['AeroTHON 2026 Mission 2  |  ' ...
           't=%.1fs  |  Alt=%.1fm  |  ' ...
           'Roll=%.1f deg  Pitch=%.1f deg'],...
           t_sim(i),pz,rad2deg(ro),rad2deg(pi_)),...
           'Color','white','FontSize',11);
    xlabel(ax,'X Forward (m)','Color','w','FontSize',10);
    ylabel(ax,'Y Lateral (m)','Color','w','FontSize',10);
    zlabel(ax,'Altitude (m)','Color','w','FontSize',10);

    annotation(fig_v,'textbox',[0.01 0.82 0.20 0.15],...
        'String',{sprintf('T:   %.1f s',t_sim(i)),...
                  sprintf('Alt: %.1f m',pz),...
                  sprintf('X:   %.1f m',px),...
                  sprintf('Y:   %.1f m',py)},...
        'Color','yellow','FontSize',10,...
        'BackgroundColor','none','EdgeColor','none',...
        'FontWeight','bold');

    drawnow limitrate;

    frame=getframe(fig_v);
    img=frame.cdata;
    if size(img,1)~=fh||size(img,2)~=fw
        img=imresize(img,[fh fw]);
    end
    writeVideo(v,img);

    if mod(fi,30)==0
        fprintf('  %.0f%% complete...\n',fi/length(frames)*100);
    end
end

close(v);
set(fig_v,'CloseRequestFcn','closereq');

fprintf('\n+------------------------------------+\n')
fprintf('| ALL FILES SAVED TO src FOLDER     |\n')
fprintf('+------------------------------------+\n')
fprintf('| Sim3_Trajectory_6panel.png        |\n')
fprintf('| Sim3_3D_FlightPath.png            |\n')
fprintf('| aerothon_mission2.mp4             |\n')
fprintf('+------------------------------------+\n')