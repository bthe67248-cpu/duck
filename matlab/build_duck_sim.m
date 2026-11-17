%% build_duck_sim.m -- builds duck_sim.slx from scratch (PDF Part 6 + Part 7)
%
%  Pulse (push_x, N) --+
%  Const (push_y = 0) -+
%  Controller (0.01 s) -> Delay -> Rate Transition -> Mux -> MuJoCo Plant (0.002 s)
%        ^                                                        |
%        +---- Rate Transition <---- Bus Selector (gyro, accel) <-+
%
if ~exist('DUCK_PROJ','var'), startup_duck; end
duck_numbers;                                    % puts HOME, K_TH, K_D ... in the workspace
mdl = 'duck_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(DUCK_PROJ,'matlab',[mdl '.slx']);
if isfile(slx), delete(slx); end
new_system(mdl);  load_system('mjLib');
set_param(mdl, 'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
               'FixedStep','DT_PLANT', 'StopTime','5', ...
               'PreLoadFcn','startup_duck; load(fullfile(DUCK_PROJ,''matlab'',''duck_params.mat''));');

%% Plant
plant = [mdl '/MuJoCo Plant'];
add_block('mjLib/MuJoCo Plant', plant, 'Position',[620 150 780 250]);
set_param(plant, 'xmlFileRel', DUCK_XML);   % its callback also sets the hidden 'xmlFile'
mo = Simulink.Mask.get(plant);
rt = mo.getParameter('renderingType');
fprintf('Plant rendering options: %s\n', strjoin(rt.TypeOptions, ', '));

%% Sensors out
add_block('simulink/Signal Routing/Bus Selector', [mdl '/Sensors'], 'Position',[840 120 850 300], ...
          'OutputSignals','gyro,accel,quat,torso_pos,q_left_knee,q_left_ankle');
add_line(mdl, 'MuJoCo Plant/1', 'Sensors/1');

add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT gyro'],  'Position',[900 130 940 150], 'OutPortSampleTime','DT_CTRL', 'Orientation','right');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT accel'], 'Position',[900 160 940 180], 'OutPortSampleTime','DT_CTRL');
add_line(mdl, 'Sensors/1', 'RT gyro/1');
add_line(mdl, 'Sensors/2', 'RT accel/1');

%% Controller (atomic, 100 Hz)
C = [mdl '/Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', C, 'Position',[150 150 300 250]);
delete_line(C, 'In1/1', 'Out1/1');
set_param([C '/In1'],  'Name','gyro',  'Position',[20 40 50 54]);
set_param([C '/Out1'], 'Name','u14',   'Position',[820 100 850 114]);
add_block('simulink/Sources/In1', [C '/accel'], 'Position',[20 100 50 114]);
add_block('simulink/Sinks/Out1',  [C '/pitch_est'], 'Position',[820 200 850 214]);
set_param(C, 'TreatAsAtomicUnit','on', 'SystemSampleTime','DT_CTRL');

% Piece 1: tilt filter (MATLAB Function block)
tf = [C '/tiltFilter'];
add_block('simulink/User-Defined Functions/MATLAB Function', tf, 'Position',[120 30 260 130]);
cfg = get_param(tf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = strjoin({
    'function [pitch, rate] = tiltFilter(gyro, acc, alpha, dt)'
    '%#codegen'
    '% Complementary filter: gyro is accurate short-term, accelerometer does not drift.'
    'persistent th'
    'if isempty(th), th = 0; end'
    'accPitch = atan2(-acc(1), acc(3));        % + = leaning forward (nose down)'
    'th    = alpha*(th + gyro(2)*dt) + (1 - alpha)*accPitch;'
    'pitch = th;         % rad'
    'rate  = gyro(2);    % rad/s'
    'end'}, newline);
add_block('simulink/Sources/Constant', [C '/alpha'], 'Value','ALPHA',   'Position',[20 150 70 170]);
add_block('simulink/Sources/Constant', [C '/dt'],    'Value','DT_CTRL', 'Position',[20 190 70 210]);
add_line(C, 'gyro/1',  'tiltFilter/1');
add_line(C, 'accel/1', 'tiltFilter/2');
add_line(C, 'alpha/1', 'tiltFilter/3');
add_line(C, 'dt/1',    'tiltFilter/4');

% Piece 2: PD on pitch -> ankle offset, saturated
add_block('simulink/Math Operations/Gain', [C '/K_TH'], 'Gain','K_TH', 'Position',[320 45 370 75]);
add_block('simulink/Math Operations/Gain', [C '/K_D'],  'Gain','K_D',  'Position',[320 95 370 125]);
add_block('simulink/Math Operations/Sum',  [C '/Sum'],  'Inputs','++', 'Position',[410 75 430 95]);
add_block('simulink/Discontinuities/Saturation', [C '/Sat'], 'UpperLimit','SAT_ANKLE', 'LowerLimit','-SAT_ANKLE', 'Position',[460 70 500 100]);
add_block('simulink/Math Operations/Gain', [C '/balance_on'], 'Gain','balance_on', 'Position',[530 70 580 100]);
% spread the scalar offset onto the two ankle entries (+left, -right: the joints are mirrored)
add_block('simulink/Math Operations/Gain', [C '/to ankles'], 'Gain','ANKLE_MAP', ...
          'Multiplication','Matrix(K*u)', 'Position',[610 65 680 105]);
add_block('simulink/Sources/Constant', [C '/home pose'], 'Value','HOME', 'Position',[610 140 680 170]);
add_block('simulink/Math Operations/Sum', [C '/Add'], 'Inputs','++', 'Position',[730 95 750 115]);
add_line(C, 'tiltFilter/1', 'K_TH/1');
add_line(C, 'tiltFilter/2', 'K_D/1');
add_line(C, 'K_TH/1', 'Sum/1');   add_line(C, 'K_D/1', 'Sum/2');
add_line(C, 'Sum/1', 'Sat/1');    add_line(C, 'Sat/1', 'balance_on/1');
add_line(C, 'balance_on/1', 'to ankles/1');
add_line(C, 'to ankles/1', 'Add/1');  add_line(C, 'home pose/1', 'Add/2');
add_line(C, 'Add/1', 'u14/1');
add_line(C, 'tiltFilter/1', 'pitch_est/1');

add_line(mdl, 'RT gyro/1',  'Controller/1', 'autorouting','on');
add_line(mdl, 'RT accel/1', 'Controller/2', 'autorouting','on');

%% Controller -> extra delay (test 5) -> rate transition -> mux with push forces -> plant
add_block('simulink/Discrete/Delay', [mdl '/Extra delay'], 'DelayLength','1', 'InitialCondition','HOME', ...
          'SampleTime','DT_CTRL', 'Position',[340 160 380 190], 'Commented','through');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT u'], 'OutPortSampleTime','DT_PLANT', ...
          'InitialCondition','HOME', 'Position',[420 160 460 190]);
add_block('simulink/Sources/Pulse Generator', [mdl '/push_x'], 'PulseType','Sample based', ...
          'Amplitude','push_F', 'Period','round(100/DT_PLANT)', 'PulseWidth','round(0.05/DT_PLANT)', ...
          'PhaseDelay','round(1/DT_PLANT)', 'SampleTime','DT_PLANT', 'Position',[420 215 460 245]);
add_block('simulink/Sources/Constant', [mdl '/push_y'], 'Value','0', 'SampleTime','DT_PLANT', 'Position',[420 265 460 285]);
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], 'Inputs','3', 'Position',[520 150 525 290]);
add_line(mdl, 'Controller/1', 'Extra delay/1');
add_line(mdl, 'Extra delay/1', 'RT u/1');
add_line(mdl, 'RT u/1', 'Mux/1');
add_line(mdl, 'push_x/1', 'Mux/2');
add_line(mdl, 'push_y/1', 'Mux/3');
add_line(mdl, 'Mux/1', 'MuJoCo Plant/1');

%% Logging (To Workspace -> out.<name> as timeseries)
logs = {'gyro',1; 'accel',2; 'quat',3; 'torso_pos',4; 'q_left_knee',5; 'q_left_ankle',6};
for i = 1:size(logs,1)
    b = [mdl '/log_' logs{i,1}];
    add_block('simulink/Sinks/To Workspace', b, 'VariableName',logs{i,1}, 'SaveFormat','Timeseries', ...
              'Position',[1000 200+35*i 1090 220+35*i]);
    add_line(mdl, sprintf('Sensors/%d', logs{i,2}), ['log_' logs{i,1} '/1'], 'autorouting','on');
end
add_block('simulink/Sinks/To Workspace', [mdl '/log_pitch_est'], 'VariableName','pitch_est', 'SaveFormat','Timeseries', 'Position',[340 300 430 320]);
add_line(mdl, 'Controller/2', 'log_pitch_est/1', 'autorouting','on');
add_block('simulink/Sinks/To Workspace', [mdl '/log_u'], 'VariableName','u', 'SaveFormat','Timeseries', 'Position',[560 320 650 340]);
add_line(mdl, 'Mux/1', 'log_u/1', 'autorouting','on');

ANKLE_MAP = zeros(14,1);  ANKLE_MAP(IDX_L_ANKLE) = 1;  ANKLE_MAP(IDX_R_ANKLE) = -1;
save(fullfile(DUCK_PROJ,'matlab','duck_params.mat'), 'ANKLE_MAP', '-append');
save_system(mdl, slx);
fprintf('Built %s\n', slx);
