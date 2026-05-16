%% drone_params.m
%% AeroTHON 2026 — Drone Parameters
%% AUW = 1.8kg | Arm = 200mm | AIR2216 KV880

%% MASS
m         = 1.800;
m_empty   = 1.700;
m_payload = 0.100;
g         = 9.81;

%% GEOMETRY
L         = 0.200;
motor_d   = L * cos(pi/4);

%% INERTIA
m_mp      = 0.082;
Ixx       = 4 * m_mp * L^2;
Iyy       = Ixx;
Izz       = Ixx * 1.2;

%% MOTOR
KV        = 880;
kf        = 1.056e-5;
km        = 1.499e-7;
w_max     = 1096;
w_hover   = 646.6;

%% PROPELLER
prop_r    = 0.127;
prop_dia  = 0.254;

%% BATTERY
V_bat     = 14.8;
C_bat     = 3.3;
E_total   = V_bat * C_bat * 3600;
E_usable  = 0.8 * E_total;
P_hover   = 480;

%% PID GAINS
Kp_rp = 6.0;  Ki_rp = 0.5;  Kd_rp = 2.0;  N_rp = 50;
Kp_y  = 4.0;  Ki_y  = 0.2;  Kd_y  = 1.0;  N_y  = 50;
Kp_z  = 3.0;  Ki_z  = 0.8;  Kd_z  = 4.0;  N_z  = 50;

%% WIND
wind_speed    = 5.0;
wind_time     = 3.0;
wind_duration = 0.5;

%% SUMMARY
fprintf('\n+--------------------------------------+\n')
fprintf('|   DRONE PARAMETERS LOADED            |\n')
fprintf('+--------------------------------------+\n')
fprintf('|  AUW:        %.3f kg               |\n', m)
fprintf('|  Arm length: %.0f mm                 |\n', L*1000)
fprintf('|  Ixx=Iyy:    %.5f kg.m2          |\n', Ixx)
fprintf('|  Izz:        %.5f kg.m2          |\n', Izz)
fprintf('|  T/W ratio:  %.2f:1                |\n', 4*1.293/m)
fprintf('|  Endurance:  ~%.1f min              |\n', E_usable/P_hover/60)
fprintf('|  Margin:     %.0fg to 2kg limit     |\n', (2.0-m)*1000)
fprintf('+--------------------------------------+\n\n')