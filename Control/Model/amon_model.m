%% AMON Lander project %%
close all;
clear all;

%%
% Parametri:
g = [0; 0; 9.81];           % Gravitacija
f_dist = 0;                 % Zunanja sila / motnja - !zanemarimo
K_d = 0;                    % Zračni upor - !zanemarimo
m = 2253;                   % Masa v gramih - !z baterijami (pazi na model baterije)
J = [0.02 0 0; 0 0.018 0; 0 0 0.03]; % Matrika vztrajnosti
r_mb = [0; 0; 200];         % OpriTrack marker premik
c_yaw = 0;
tau_edf = 0.2;              % delay edf-ja - !nastavi
tau_servo = 0.2;            % delay servota - !nastavi
alpha_T = 0;                % preslikava PWM - thrust
t_d = 0;                    % skupna zakasnitev med ukazom in efektom

% Omejitve vhodov
EDF_min = 0;                % procenti
EDF_max = 100;              % procenti
servo_min = -45;            % stopinje
servo_max = -45;            % stopinje

% Servo offseti [rad]
servo1_offset = 0;
servo2_offset = 0;
servo3_offset = 0;
servo4_offset = 0;

f_0_T = [0; 0; 0];          % Default sila (offset EDF-ja na drona)
r_T = [0; 0; 0];            % Offset EDF-ja iz CoM
tau_0_T = [0; 0; 0];        % Default navor (offset EDF-ja na drona)

% Položaj finov glede na CoM [m] (*z merjen od CoM do osi servotov)
% zaporednje: x+, x-, y+, y-
r_fins = [   0.50  -0.50      0      0; 
                0      0   0.50  -0.50; 
           -0.115 -0.115 -0.115 -0.115]; 

% Smerni vektorji (smer sile pri pozitivnem odklonu servota)
% zaporednje: x+, x-, y+, y-
n_fins = [0  0  1 -1; 
          1 -1  0  0;
          0  0  0  0]; 
k_fins = [1 1 1 1];         % Koeficient učinkovitosti fina (k premajhen - dron ne reagira, k prevelik - dron preveč agresiven)

% Vektor krmilnih vhodov: EDF[%], X+[deg], X-[deg], Y+[deg], Y-[deg]
edf_cmd = 0;
servo1_cmd = 0;
servo2_cmd = 0;
servo3_cmd = 0;
servo4_cmd = 0;

u = [0, edf_cmd, servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd];


%% Karakterizacija EDF motorja

EDF_sequence1 = [0:5:100, 100:-5:0];
time_vector   = (0:length(EDF_sequence1)-1) * 3;  % vsak korak 3s
T_end = length(EDF_sequence1)*3; 

clc;
out1 = sim("amon_model_EDF.slx");

t_u   = out1.u.Time;
t_T   = out1.T.Time;
t_tau = out1.tau.Time;

u_data   = out1.u.Data;
T_data   = out1.T.Data;
tau_data = out1.tau.Data;

figure;
% --- Top subplot: Input stair signal ---
subplot(3,1,1);
stairs(t_u, u_data, 'b', 'LineWidth', 1.5);
ylabel('u [%]');
title('EDF Stair Test');
grid on;
ylim([-5, 105]);

% --- Middle subplot: Thrust T ---
subplot(3,1,2);
plot(t_T, T_data, 'r', 'LineWidth', 1.5);
ylabel('T [N]');
grid on;

% --- Bottom subplot: Torque tau ---
subplot(3,1,3);
plot(t_tau, tau_data, 'g', 'LineWidth', 1.5);
ylabel('\tau [Nm]');
xlabel('Time [s]');
grid on;

linkaxes(findall(gcf,'Type','axes'), 'x');  % sync x-axis zoom/pan

%% Matrike vplivov (iz 3D modela) - B_F, B_tau
B_F = zeros(3,4);
B_tau = zeros(3,4);

for i = 1:4
    B_F(:,i) = k(i) * n(:,i);
    B_tau(:,i) = cross(r(:,i), BF(:,i));
end

% Linearni način
%F_fins_lin = T * BF * delta_eff(:);
%tau_fins_lin = T * Btau * delta_eff(:);