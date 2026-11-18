%% duck_numbers.m -- hand calculations (PDF Part 4) + balance gains (Part 7)
% Geometry/mass come from model/stand_numbers.json, which
%   .venv\Scripts\python.exe scripts\stand_numbers.py model\duck_v3_scene.xml
% writes from the MuJoCo model. Only td is typed in by hand.
if ~exist('DUCK_PROJ','var'), startup_duck; end
S  = jsondecode(fileread(fullfile(DUCK_PROJ,'model','stand_numbers.json')));

if isfield(S,'model_scene_sha256')
    MODEL_REV = S.model_scene_sha256;      % which generated scene these numbers describe (scripts/stand_numbers.py)
else
    MODEL_REV = 'unknown (stand_numbers.json predates provenance fields)';
end
if isfield(S,'mass_status'), MASS_STATUS = S.mass_status; else, MASS_STATUS = 'unknown'; end
disp(['model ' MODEL_REV ' | ' MASS_STATUS]);
if isfield(S,'printed_part_mass_g')
    fprintf(['printed parts (g): upper head %.1f, lower head %.1f, rear shield %.1f  [replace with slicer/scale values]' newline], ...
            S.printed_part_mass_g.duck_head_upper_v3, S.printed_part_mass_g.duck_head_lower_v3, S.printed_part_mass_g.hat_rear_shield_v3);
end
m  = S.mass;        % kg   whole robot
h  = S.h;           % m    centre of mass above the ankle pivot, standing
a  = S.a_toe;       % m    centre of mass to the front edge of the sole
ab = S.a_heel;      % m    centre of mass to the rear edge of the sole
d  = S.d_knee;      % m    horizontal distance knee pivot to centre-of-mass line
td = 0.03;          % s    servo command-to-motion delay  (EXAMPLE until the bench test)
if isfield(S,'servo_delay_s') && ~isempty(S.servo_delay_s)
    td = S.servo_delay_s;
    fprintf('Measured servo delay %.4f s (delay check only; configure plant latency separately).\n', td);
end
kp_servo = S.kp;    % N.m/rad  one servo's position gain in the model
stall    = S.stall; % N.m
g  = 9.81;

tau_knee  = m*g*d;
p         = sqrt(g/h);
t_double  = log(2)/p;
wc        = 2*p;
f_loop    = 20*wc/(2*pi);
delay_chk = p*td;
theta_max = rad2deg(a/h);
J_max_fwd = m*p*a;
J_max_bwd = m*p*ab;

fprintf('\n== Hand calculations ==\n');
fprintf('mass                 %.3f kg\n', m);
fprintf('CoM height h         %.3f m   (above ankle pivot)\n', h);
fprintf('knee holding torque  %.3f N.m  (%.0f%% of stall %.2f N.m; rule: under 33%%)\n', tau_knee, 100*tau_knee/stall, stall);
fprintf('fall rate p          %.2f rad/s\n', p);
fprintf('doubling time        %.0f ms\n', 1000*t_double);
fprintf('needed bandwidth wc  %.1f rad/s = %.1f Hz\n', wc, wc/2/pi);
fprintf('minimum loop rate    %.0f Hz  (controller runs at 100 Hz)\n', f_loop);
fprintf('delay check p*td     %.2f  (below 0.3 comfortable, above 0.5 very hard)\n', delay_chk);
fprintf('max recoverable tilt %.1f deg\n', theta_max);
fprintf('max push forward     %.3f N.s  (= %.1f N for 0.05 s)\n', J_max_fwd, J_max_fwd/0.05);
fprintf('max push backward    %.3f N.s  (= %.1f N for 0.05 s)\n', J_max_bwd, J_max_bwd/0.05);

%% PD ankle gains: I*thdd = m g h th - tau,  tau = Kp th + Kd thd
% The PDF's rule (bandwidth 2*p, damping 0.7) gives 4.21 / 0.26 and passes every test except
% extra delay: one more 10 ms sample makes it oscillate and fall after a push.
% scripts/tune_gains.py sweeps gains against delay; everything from K_TH 1.5 to 2.5 survives
% 4 extra samples, so the loop is retuned to the top of that robust band:
BW_FACTOR = 1.4;               % closed-loop bandwidth = BW_FACTOR * p   (PDF rule: 2)
ZETA      = 0.77;              % damping ratio                           (PDF rule: 0.7)
wc_ctrl = BW_FACTOR*p;
I   = m*h^2;
Kp  = I*wc_ctrl^2 + m*g*h;     % N.m/rad
Kd  = 2*ZETA*wc_ctrl*I;        % N.m.s/rad
kp_total = 2*kp_servo;         % two ankle servos share the load
K_TH = Kp/kp_total;            % ankle rad per rad of pitch
K_D  = Kd/kp_total;            % ankle rad per rad/s of pitch rate
sag_knee_deg = rad2deg(tau_knee/2/kp_servo);   % two legs share the weight
fprintf('\n== PD ankle gains ==\n');
fprintf('Kp = %.2f N.m/rad, Kd = %.3f N.m.s/rad\n', Kp, Kd);
fprintf('ankle offset = %.2f*pitch + %.3f*pitch_rate   (rad)\n', K_TH, K_D);
fprintf('expected knee sag (both feet down) %.1f deg\n', sag_knee_deg);

%% LQR upgrade (PDF Part 7): states x, xdot, foot pressure point; servo lag T
w = p;  T = 0.05;
A = [0 1 0;  w^2 0 -w^2;  0 0 -1/T];
B = [0; 0; 1/T];
[K_lqr, ~, poles_lqr] = lqr_base(A, B, diag([100 1 0]), 1);
fprintf('\n== LQR (base-MATLAB solver) ==\n');
fprintf('K = [%.2f  %.2f  %.2f]\n', K_lqr);
fprintf('closed-loop poles: %s\n', mat2str(round(poles_lqr.',1)));
% sanity check against the PDF example (h = 0.12): expect K = [-13.61 -1.43 2.56]
w0 = sqrt(9.81/0.12);
K_chk = lqr_base([0 1 0; w0^2 0 -w0^2; 0 0 -1/T], B, diag([100 1 0]), 1);
fprintf('PDF check (h=0.12): K = [%.2f  %.2f  %.2f]  (PDF says [-13.61 -1.43 2.56])\n', K_chk);

%% Controller parameters used by duck_sim.slx
HOME        = S.home(:);       % 14 joint targets, rad, actuator order
DT_CTRL     = 0.01;            % s, like the STM32 loop
DT_PLANT    = S.timestep;      % s, MuJoCo step
ALPHA       = 0.5/(0.5+DT_CTRL);
SAT_ANKLE   = 0.25;            % rad
IDX_L_ANKLE = 5;  IDX_R_ANKLE = 14;   % 1-based actuator indices
balance_on  = 1;               % 0 = hold pose only
ANKLE_MAP  = zeros(14,1);  ANKLE_MAP(IDX_L_ANKLE) = 1;  ANKLE_MAP(IDX_R_ANKLE) = -1;   % used by duck_sim.slx (also saved here so re-running this script never drops it)
push_F      = 0;               % N, push along +x (forward) at t = 1 s for 0.05 s
save(fullfile(DUCK_PROJ,'matlab','duck_params.mat'), 'HOME','DT_CTRL','DT_PLANT','ALPHA', ...
     'SAT_ANKLE','K_TH','K_D','balance_on','push_F','J_max_fwd','J_max_bwd','sag_knee_deg','K_lqr', ...
     'MODEL_REV','MASS_STATUS','ANKLE_MAP');
