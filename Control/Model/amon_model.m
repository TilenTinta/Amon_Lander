%% AMON Lander project %%
close all;
clear all;

%%
% Parametri:
g = [0; 0; 9.81];           % Gravitacija
f_dist = 0;                 % Zunanja sila / motnja - !zanemarimo
K_d = 0;                    % Zračni upor - !zanemarimo
m = 2253;                   % Masa v gramih - !z baterijami (pazi na model baterije)
J = [0; 0; 0];              % Matrika vztrajnosti
r_mb = 0;                   % OpriTrack marker premik
K_d = 0;
K_F = 0;
B_tau = 0;
c_yaw = 0;
tau_edf = 0;
tau_servo = 0;
delta_0 = 0;                % ničelni koti servo motorjev - nevtralni offfset
alpha_T = 0;                % preslikava PWM - thrust
t_d = 0;



% Omejitve vhodov
EDF_min = 0;            % procenti
EDF_max = 100;          % procenti
servo_min = -45;        % stopinje
servo_max = -45;        % stopinje

% Servo offseti
servo1_offset = 0;
servo2_offset = 0;
servo3_offset = 0;
servo4_offset = 0;

% Matrika vplivov
B_F = [0 0 0 0; 0 0 0 0; 0 0 0 0];


f_0_T = [0; 0; 0];  % Default sila (offset EDF-ja na drona)
r_T = [0; 0; 0];    % Offset EDF-ja iz CoM
tau_0_T = [0; 0; 0];% Default navor (offset EDF-ja na drona)
r_fins = [ 0.08 -0.08 0 0 ; 0 0 0.08 -0.08; 0 0 0 0]; % Položaj finov glede na CoM [m] 
n_fins = [0 0 1 -1; 1 -1 0 0; 0 0 0 0]; % Smerni vektorji (kalibracija iz meritev)
k_fins = [0.5 0.5 0.5 0.5]; % Koeficient učinkovitosti fina (k premajhen - dron ne reagira, k prevelik - dron preveč agresiven)

% Vektor krmilnih vhodov: EDF[%], X+[deg], X-[deg], Y+[deg], Y-[deg]
edf_cmd = 0;
servo1_cmd = 0;
servo2_cmd = 0;
servo3_cmd = 0;
servo4_cmd = 0;

u = [0, edf_cmd, servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd];


% Razširjen vektor stanj:
p = [0; 0; 0];              % px, py, pz
v = [0; 0; 0];              % vx, vy, vz
q = [0; 0; 0; 0];           % q0, q1, q2, q3
w = [0; 0; 0];              % wx, wy, wz
T = 0;
delta_fin = [0; 0; 0; 0];   % d_xp, d_xn, d_yp, d_yn

x = transpose([p; v; q; w; T; delta_fin]);







%% Karakterizacija EDF motorja
T_end = 14000; 
dt = 2;          
t = 0:dt:Tend; % cel čas

step = 5;
for pwm = 0:step:100
    disp(pwm);
end



