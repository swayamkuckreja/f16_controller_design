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
%% OPEN LOOP ANALYSIS: CALCULATION & PRINTING
%% =======================================================================
% 1. Reduces the model (removes actuator dynamics).
% 2. Calculates Eigenvalues for all modes.
% 3. Prints natural frequency, damping, period, time constant, and T1/2.

fprintf('\n==============================================================\n');
fprintf('       INHERENT MOTION CHARACTERISTICS (Computed)             \n');
fprintf('       Condition: Alt = %.0f ft, Vel = %.0f ft/s\n', altitude, velocity);
fprintf('==============================================================\n');

%% 1. Model Reduction (Equation 5.1)
% ------------------------------------------------------------------
% Extract bare airframe dynamics by moving Actuator Columns from A to B.

% Longitudinal Indices (Vt, alpha, theta, q)
idx_lon_aero = [7 8 5 11]; 
idx_lon_act  = [13 14]; % Thrust, Elevator

% Lateral Indices (beta, phi, p, r)
idx_lat_aero = [9 4 10 12];
idx_lat_act  = [15 16]; % Aileron, Rudder

% Create Reduced Matrices
A_lon = SS_lo.A(idx_lon_aero, idx_lon_aero);
B_lon = SS_lo.A(idx_lon_aero, idx_lon_act); % Actuator effect -> Input
sys_lon_red = ss(A_lon, B_lon, eye(4), zeros(4,2));

A_lat = SS_lo.A(idx_lat_aero, idx_lat_aero);
B_lat = SS_lo.A(idx_lat_aero, idx_lat_act);
sys_lat_red = ss(A_lat, B_lat, eye(4), zeros(4,2));

%% 2. Eigenvalue Identification
% ------------------------------------------------------------------

% --- Longitudinal Modes ---
eigs_lon = eig(A_lon);
% Sort by Frequency (Imaginary part): Phugoid is Slow, Short Period is Fast
[~, idx_lon] = sort(abs(imag(eigs_lon)));
phugoid_poles = eigs_lon(idx_lon(1:2));      % Low Freq
short_period_poles = eigs_lon(idx_lon(3:4)); % High Freq

% --- Lateral Modes ---
eigs_lat = eig(A_lat);
% Separate Real and Complex
is_complex = abs(imag(eigs_lat)) > 0.0001;

dutch_roll_poles = eigs_lat(is_complex);
real_poles = sort(eigs_lat(~is_complex)); 

% Sort Real poles by Magnitude
% Spiral is close to 0 (Smallest Magnitude)
% Roll is large negative (Largest Magnitude)

[~, idx_real] = sort(abs(real_poles)); % Sort by magnitude ascending
spiral_pole = real_poles(idx_real(1));   % Smallest abs value (e.g. -0.008)
roll_pole   = real_poles(idx_real(end)); % Largest abs value (e.g. -1.44)

%% 3. Calculation & Printing Functions
% ------------------------------------------------------------------

% --- Helper Function for Periodic Modes (Phugoid, Short Period, Dutch Roll) ---
% P = 2*pi / wd
% T1/2 = ln(2) / abs(real)
print_periodic = @(name, p) fprintf(...
    ['%-15s\n' ...
     '  Eigenvalues:  %8.4f +/- %.4fi\n' ...
     '  omega_n:      %8.4f [rad/s]\n' ...
     '  zeta:         %8.4f [-]\n' ...
     '  Period (P):   %8.4f [s]\n' ...
     '  T_1/2:        %8.4f [s]\n\n'], ...
     name, real(p(1)), abs(imag(p(1))), ...
     abs(p(1)), ...                          % wn
     -real(p(1))/abs(p(1)), ...              % zeta
     (2*pi)/abs(imag(p(1))), ...             % Period
     log(2)/abs(real(p(1))) ...              % T1/2
);

% --- Helper Function for Aperiodic Modes (Spiral, Roll) ---
% tau = 1 / abs(real)
% T1/2 = ln(2) / abs(real)
print_aperiodic = @(name, p) fprintf(...
    ['%-15s\n' ...
     '  Eigenvalue:   %8.4f\n' ...
     '  Time Const(tau): %8.4f [s]\n' ...
     '  T_1/2:        %8.4f [s]\n\n'], ...
     name, real(p), ...
     1/abs(real(p)), ...                     % Tau
     log(2)/abs(real(p)) ...                 % T1/2
);

%% 4. Execute Printing
% ------------------------------------------------------------------

print_periodic('PHUGOID', phugoid_poles);
print_periodic('SHORT PERIOD', short_period_poles);
print_periodic('DUTCH ROLL', dutch_roll_poles);
print_aperiodic('APERIODIC ROLL', roll_pole);

% Special check for Spiral (Stable vs Unstable)
fprintf('SPIRAL MODE\n');
fprintf('  Eigenvalue:   %8.4f\n', real(spiral_pole));
if real(spiral_pole) > 0
    % Unstable
    t_double = log(2)/abs(real(spiral_pole));
    fprintf('  STABILITY:    UNSTABLE\n');
    fprintf('  Time to Double: %8.4f [s]\n\n', t_double);
else
    % Stable
    fprintf('  STABILITY:    STABLE\n');
    fprintf('  Time Const(tau): %8.4f [s]\n', 1/abs(real(spiral_pole)));
    fprintf('  T_1/2:        %8.4f [s]\n\n', log(2)/abs(real(spiral_pole)));
end

fprintf('==============================================================\n');

%% =======================================================================
%% 6. PRINT MATRICES IN EQUATION FORMAT
%% =======================================================================

fprintf('\n\n==============================================================\n');
fprintf('       STATE SPACE MATRICES (Reference Format)                \n');
fprintf('==============================================================\n');

%% --- Longitudinal Equation Printout ---
% Image shows inputs: [delta_e] only.
% Our B_lon has [Thrust, Elevator]. We select Column 2 for Elevator.
B_lon_print = B_lon(:, 2); 

fprintf('\nLongitudinal Direction:\n');
fprintf('States: [Vt, alpha, theta, q]''\n');
fprintf('Input:  [delta_el]\n\n');

% Row 1 (Vt_dot)
fprintf('[ Vt_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [ Vt ]   [ %9.4f ]\n', ...
    A_lon(1,1), A_lon(1,2), A_lon(1,3), A_lon(1,4), B_lon_print(1));
% Row 2 (alpha_dot)
fprintf('[ al_dot ] = [ %9.4f %9.4f %9.4f %9.4f ] [ al ] + [ %9.4f ] [ delta_el ]\n', ...
    A_lon(2,1), A_lon(2,2), A_lon(2,3), A_lon(2,4), B_lon_print(2));
% Row 3 (theta_dot)
fprintf('[ th_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [ th ]   [ %9.4f ]\n', ...
    A_lon(3,1), A_lon(3,2), A_lon(3,3), A_lon(3,4), B_lon_print(3));
% Row 4 (q_dot)
fprintf('[  q_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [  q ]   [ %9.4f ]\n', ...
    A_lon(4,1), A_lon(4,2), A_lon(4,3), A_lon(4,4), B_lon_print(4));


%% --- Lateral Equation Printout ---
% Image shows inputs: [delta_r; delta_a] (Rudder top, Aileron bottom).
% Our B_lat has [Aileron, Rudder]. We must swap columns to match image.
B_lat_print = [B_lat(:, 2), B_lat(:, 1)]; 

fprintf('\n\nLateral Direction:\n');
fprintf('States: [beta, phi, p, r]''\n');
fprintf('Inputs: [delta_r; delta_a]\n\n');

% Row 1 (beta_dot)
fprintf('[ be_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [ be ]   [ %9.4f %9.4f ]\n', ...
    A_lat(1,1), A_lat(1,2), A_lat(1,3), A_lat(1,4), B_lat_print(1,1), B_lat_print(1,2));
% Row 2 (phi_dot)
fprintf('[ ph_dot ] = [ %9.4f %9.4f %9.4f %9.4f ] [ ph ] + [ %9.4f %9.4f ] [ delta_r ]\n', ...
    A_lat(2,1), A_lat(2,2), A_lat(2,3), A_lat(2,4), B_lat_print(2,1), B_lat_print(2,2));
% Row 3 (p_dot)
fprintf('[  p_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [  p ]   [ %9.4f %9.4f ] [ delta_a ]\n', ...
    A_lat(3,1), A_lat(3,2), A_lat(3,3), A_lat(3,4), B_lat_print(3,1), B_lat_print(3,2));
% Row 4 (r_dot)
fprintf('[  r_dot ]   [ %9.4f %9.4f %9.4f %9.4f ] [  r ]   [ %9.4f %9.4f ]\n', ...
    A_lat(4,1), A_lat(4,2), A_lat(4,3), A_lat(4,4), B_lat_print(4,1), B_lat_print(4,2));

fprintf('\n==============================================================\n');


%% =======================================================================
%% 7. DYNAMICS ANALYSIS (Separated Graphs, No Numbering)
%% =======================================================================
% Inputs:
% - Short Period & Phugoid: STEP Input (-1 deg)
% - Dutch Roll, Roll, Spiral: IMPULSE Input (5 deg for 1 sec)
% Style: White background, Black text, All Blue Lines.
% Layout: Separated plots for coupled states.

% --- TIME VECTORS ---
t_short = 0:0.01:10;   % Short Period / Roll
t_med   = 0:0.01:20;   % Dutch Roll
t_long  = 0:0.1:300;   % Phugoid / Spiral

% --- INPUT DEFINITIONS ---
mag_step = 1; % 1 degree step
mag_imp  = 5; % 5 degree impulse

% 1. Elevator STEP (-1 deg) -> For Longitudinal
u_ele_step_short = zeros(length(t_short), 2); u_ele_step_short(:,2) = -mag_step;
u_ele_step_long  = zeros(length(t_long), 2);  u_ele_step_long(:,2)  = -mag_step;

% 2. Rudder IMPULSE (5 deg for 1 second) -> For Dutch Roll
u_rud_imp = zeros(length(t_med), 2);
u_rud_imp(t_med <= 1.0, 2) = mag_imp; 

% 3. Aileron IMPULSE (5 deg for 1 second) -> For Roll & Spiral
u_ail_imp_short = zeros(length(t_short), 2);
u_ail_imp_short(t_short <= 1.0, 1) = mag_imp;

u_ail_imp_long = zeros(length(t_long), 2);
u_ail_imp_long(t_long <= 1.0, 1) = mag_imp;

% --- SIMULATIONS ---
[y_sp, ~]   = lsim(sys_lon_red, u_ele_step_short, t_short);
[y_ph, ~]   = lsim(sys_lon_red, u_ele_step_long, t_long);
[y_dr, ~]   = lsim(sys_lat_red, u_rud_imp, t_med);
[y_roll, ~] = lsim(sys_lat_red, u_ail_imp_short, t_short);
[y_spi, ~]  = lsim(sys_lat_red, u_ail_imp_long, t_long);


% --- PLOTTING ---
% Set figure background to white, size large enough for 4x2 grid
fig = figure('Name', 'Separated Dynamics Analysis', 'Color', 'w', 'Position', [50 50 1200 1000]);

% Default settings for black text
set(fig, 'DefaultAxesXColor', 'k', 'DefaultAxesYColor', 'k', 'DefaultTextColor', 'k');

% --- ROW 1: Fast Modes (Short Period & Aperiodic Roll) ---

% Short Period (Pitch Rate)
subplot(4,2,1);
plot(t_short, y_sp(:,4), 'b', 'LineWidth', 2); 
title('Short Period (Elevator Step)', 'Color', 'k');
ylabel('Pitch Rate q [deg/s]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% Aperiodic Roll (Roll Rate)
subplot(4,2,2);
plot(t_short, y_roll(:,3), 'b', 'LineWidth', 2); 
title('Aperiodic Roll (Aileron Impulse)', 'Color', 'k');
ylabel('Roll Rate p [deg/s]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% --- ROW 2: Phugoid (Separated) ---

% Phugoid (Pitch Angle)
subplot(4,2,3);
plot(t_long, y_ph(:,3), 'b', 'LineWidth', 2); 
title('Phugoid Pitch (Elevator Step)', 'Color', 'k');
ylabel('Pitch Angle \theta [deg]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% Phugoid (Velocity)
subplot(4,2,4);
plot(t_long, y_ph(:,1), 'b', 'LineWidth', 2); 
title('Phugoid Velocity (Elevator Step)', 'Color', 'k');
ylabel('Velocity V_T [ft/s]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% --- ROW 3: Dutch Roll (Separated) ---

% Dutch Roll (Sideslip)
subplot(4,2,5);
plot(t_med, y_dr(:,1), 'b', 'LineWidth', 2); 
title('Dutch Roll Sideslip (Rudder Impulse)', 'Color', 'k');
ylabel('Sideslip \beta [deg]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% Dutch Roll (Yaw Rate)
subplot(4,2,6);
plot(t_med, y_dr(:,4), 'b', 'LineWidth', 2); 
title('Dutch Roll Yaw Rate (Rudder Impulse)', 'Color', 'k');
ylabel('Yaw Rate r [deg/s]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% --- ROW 4: Spiral Mode ---

% Spiral Mode (Bank Angle)
subplot(4,2,7);
plot(t_long, y_spi(:,2), 'b', 'LineWidth', 2); 
title('Spiral Mode (Aileron Impulse)', 'Color', 'k');
ylabel('Bank Angle \phi [deg]', 'Color', 'k'); xlabel('Time [s]', 'Color', 'k');
grid on; set(gca, 'Color', 'w');

% Global Title
sgtitle(['Eigenmotion Responses (Alt: ' num2str(altitude) ' ft, Vel: ' num2str(velocity) ' ft/s)'], 'Color', 'k');