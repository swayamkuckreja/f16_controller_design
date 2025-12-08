%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
% Edit: Ewoud Smeur (2021)
%================================================
clear;

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
altitude = input('Enter the altitude for the simulation (ft)  :  ');
velocity = input('Enter the velocity for the simulation (ft/s):  ');

FC_flag = 1; % Trim for steady wings-level flight

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the hifi model at the desired alt and vel.
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_hi = linearize('LIN_F16Block');

disp(' ');
%% Find trim for lofi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('LIN_F16Block');


%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Direction
%%%%%%%%%%%%%%%%%%%%%%%

long_states = [3 5 7 8 11 13 14];
long_inputs = [1 2];
long_outputs = [3 5 7 8 11];

SS_long_lo = ss(SS_lo.A(long_states,long_states), SS_lo.B(long_states,long_inputs), SS_lo.C(long_outputs,long_states), SS_lo.D(long_outputs,long_inputs));
SS_long_hi = ss(SS_hi.A(long_states,long_states), SS_hi.B(long_states,long_inputs), SS_hi.C(long_outputs,long_states), SS_hi.D(long_outputs,long_inputs));

SS_long_lo.StateName = SS_lo.StateName(long_states);
SS_long_hi.StateName = SS_hi.StateName(long_states);

SS_long_lo.InputName= SS_lo.InputName(long_inputs);
SS_long_hi.InputName= SS_hi.InputName(long_inputs);

%%%%%%%%%%%%%%%%%%%%
%% Lateral Direction
%%%%%%%%%%%%%%%%%%%%

lat_states = [4 6 7 9 10 12 13 15 16];
lat_inputs = [1 3 4];
lat_outputs = [4 6 7 9 10 12];

SS_lat_lo = ss(SS_lo.A(lat_states,lat_states), SS_lo.B(lat_states,lat_inputs), SS_lo.C(lat_outputs,lat_states), SS_lo.D(lat_outputs,lat_inputs));
SS_lat_hi = ss(SS_hi.A(lat_states,lat_states), SS_hi.B(lat_states,lat_inputs), SS_hi.C(lat_outputs,lat_states), SS_hi.D(lat_outputs,lat_inputs));

SS_lat_lo.StateName = SS_lo.StateName(lat_states);
SS_lat_hi.StateName = SS_hi.StateName(lat_states);

SS_lat_lo.InputName= SS_lo.InputName(lat_inputs);
SS_lat_hi.InputName= SS_hi.InputName(lat_inputs);


%% All Poles
figure(1); 
pzmap(SS_hi, 'r', SS_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Longitudinal Poles
%%
figure(2); 
pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Lateral Poles
%%
figure(3); 
pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Bode plot longitudinal 

% Choose an input and output
input = 2;
output = 3;

omega = logspace(-2,2,100);

figure
bode(SS_long_hi(output,input),omega)
hold on;
bode(SS_long_lo(output,input),omega)
legend('hifi','lofi')

%% Bode plot lateral 

% Choose an input and output
input = 2;
output = 3;

omega = logspace(-2,2,100);

figure
bode(SS_lat_hi(output,input),omega)
hold on;
bode(SS_lat_lo(output,input),omega)
legend('hifi','lofi')

%% =======================================================================
%% CHAPTER 7: Pitch Rate Command System Design
%% =======================================================================
disp('====================================================');
disp('CHAPTER 7: Pitch Rate Command System Design');
disp('====================================================');

% -------------------------------------------------------------------------
% 1. Reduce Model to Short Period (Alpha, Q) without Actuator Dynamics
% -------------------------------------------------------------------------
% Based on your long_states definition: [alt, theta, vt, alpha, q, thrust, elev]
% Indices in SS_long_lo:
% 1: h, 2: theta, 3: vt, 4: alpha, 5: q, 6: thrust_state, 7: elev_state

idx_alpha = 4;
idx_q     = 5;
idx_de    = 7; % The state corresponding to elevator deflection

% A_sp: The aerodynamic interaction between alpha and q
A_sp = SS_long_lo.A([idx_alpha, idx_q], [idx_alpha, idx_q]);

% B_sp: The effect of elevator deflection (state 7) on alpha and q.
% We extract this from the A-matrix column because the actuator is a state 
% in the full model, but acts as the input in the reduced model.
B_sp = SS_long_lo.A([idx_alpha, idx_q], idx_de);

% C_sp: We want to measure alpha and q
C_sp = eye(2); 

% D_sp: No direct feedthrough
D_sp = [0; 0];

% Create State Space Object
SS_sp = ss(A_sp, B_sp, C_sp, D_sp);
SS_sp.StateName = {'alpha', 'q'};
SS_sp.InputName = {'delta_e'};
SS_sp.OutputName = {'alpha', 'q'};

% -------------------------------------------------------------------------
% 2. Print the Reduced Matrices (As requested)
% -------------------------------------------------------------------------
disp('--- Reduced Short Period Model Matrices (2-state) ---');
disp('A_sp (alpha, q):');
disp(A_sp);
disp('B_sp (input: delta_e):');
disp(B_sp);
disp('C_sp:');
disp(C_sp);
disp('D_sp:');
disp(D_sp);

% -------------------------------------------------------------------------
% 3. Define Design Requirements (CAP & Gibson)
% -------------------------------------------------------------------------
% Requirements from assignment PDF:
% wn_sp = 0.03 * V
% zeta_sp = 0.5
% 1/T_theta2 = 0.75 * wn_sp

req_wn = 0.03 * velocity; 
req_zeta = 0.5;

fprintf('Design Requirements at V = %.1f ft/s:\n', velocity);
fprintf('  Required Natural Frequency (wn): %.4f rad/s\n', req_wn);
fprintf('  Required Damping Ratio (zeta):   %.2f\n', req_zeta);

% Calculate desired closed-loop pole locations
% s = -zeta*wn +/- i*wn*sqrt(1-zeta^2)
p1 = -req_zeta * req_wn + 1i * req_wn * sqrt(1 - req_zeta^2);
p2 = -req_zeta * req_wn - 1i * req_wn * sqrt(1 - req_zeta^2);
desired_poles = [p1; p2];

% -------------------------------------------------------------------------
% 4. Pole Placement (Calculate Feedback K)
% -------------------------------------------------------------------------
% Place poles for A - B*K
K = place(A_sp, B_sp, desired_poles);

K_alpha = K(1);
K_q     = K(2);

fprintf('\nCalculated Feedback Gains:\n');
fprintf('  K_alpha: %.4f\n', K_alpha);
fprintf('  K_q:     %.4f\n', K_q);

% -------------------------------------------------------------------------
% 5. Design Lead-Lag Prefilter (T_theta2 adjustment)
% -------------------------------------------------------------------------
% We need to replace the old T_theta2 zero with the required one.
% Required: 1/T_new = 0.75 * wn_sp
inv_T_new = 0.75 * req_wn;

% Find Current Zero (1/T_old)
% This is the zero of the transfer function from delta_e to q
[z_open, ~] = tzero(SS_sp(2,1)); 
inv_T_old = -real(z_open(1)); % Zeros are usually negative real

fprintf('\nPrefilter Design:\n');
fprintf('  Canceling old zero at: %.4f\n', -inv_T_old);
fprintf('  Placing new zero at:   %.4f\n', -inv_T_new);

% Prefilter TF: (s + 1/T_old) / (s + 1/T_new)
prefilter = tf([1, inv_T_old], [1, inv_T_new]);

% -------------------------------------------------------------------------
% 6. Calculate Feedforward Gain (N)
% -------------------------------------------------------------------------
% Build the Closed Loop System (A - BK)
A_cl = A_sp - B_sp * K;
SS_cl = ss(A_cl, B_sp, C_sp, D_sp); 

% We look at the Pitch Rate (q) output (2nd output)
SS_cl_q = SS_cl(2,1); 

% Combined system (Prefilter * ClosedLoop)
Sys_combined = series(prefilter, SS_cl_q);

% Find DC gain to scale steady state to 1
dc_val = dcgain(Sys_combined);
N = 1 / dc_val;

fprintf('  Feedforward Gain N:    %.4f\n', N);

% -------------------------------------------------------------------------
% 7. Final System & Verification Plot
% -------------------------------------------------------------------------
Final_System = N * Sys_combined;

figure(4);
step(Final_System);
grid on;
title(sprintf('Step Response of Pitch Rate Command System\n(V=%.0f ft/s, Alt=%.0f ft)', velocity, altitude));
ylabel('Pitch Rate q (rad/s)');
legend('Designed Controller Response');

% Check CAP Value
% CAP = (g * wn^2 * T_theta2) / V
g = 32.174;
T_theta2_design = 1 / inv_T_new;
CAP = (g * req_wn^2 * T_theta2_design) / velocity;

fprintf('\nVerification:\n');
fprintf('  Calculated CAP: %.4f\n', CAP);