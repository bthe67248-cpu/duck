%% run_tests.m -- PDF Part 8 tests, run on duck_sim.slx (Simulink controller + MuJoCo plant)
% Usage:  startup_duck; run_tests            (needs duck_sim.slx; build_duck_sim makes it)
% Results print to the command window and go to results/test_results.txt + PNG plots.
if ~exist('DUCK_PROJ','var'), startup_duck; end
load(fullfile(DUCK_PROJ,'matlab','duck_params.mat'));
mdl = 'duck_sim';  load_system(mdl);
plant = [mdl '/MuJoCo Plant'];  dly = [mdl '/Extra delay'];
set_param(plant, 'renderingType', 'None');            % no window: tests run much faster
resDir = fullfile(DUCK_PROJ,'results');  if ~isfolder(resDir), mkdir(resDir); end
fid = fopen(fullfile(resDir,'test_results.txt'),'w');
logf = @(varargin) logBoth(fid, varargin{:});
logf('Duckling V3 Simulink+MuJoCo tests, %s\n', datestr(now));
logf('gains: ankle = %.2f*pitch + %.3f*rate\n\n', K_TH, K_D);

%% 1. Hold pose, balance loop off
r = runOne(mdl, 0, 0, 10);
logf('1. Hold pose, loop OFF, 10 s : %s', verdict(r));
if r.fell, logf(' (tips over at t = %.2f s)', r.tFall); end
logf('\n   knee sag at t = 0.25 s: %.1f deg (hand calc %.1f deg)\n', r.kneeSag, sag_knee_deg);

%% 2. Stand, loop on, 60 s
r2 = runOne(mdl, 1, 0, 60);
logf('2. Stand, loop ON, 60 s      : %s | pitch mean %+.2f deg, band +/- %.2f deg around it (after 2 s)\n', ...
     verdict(r2), r2.pitchMean, r2.pitchBand);
plotRun(r2, 'Test 2: stand, balance loop on', fullfile(resDir,'test2_stand.png'));

%% 3. Small push 2 N x 0.05 s = 0.10 N.s
r3 = runOne(mdl, 1, 2, 6);
logf('3. Push 2 N x 0.05 s (0.10 N.s): %s | peak pitch %.1f deg, back within 1 deg after %.2f s\n', ...
     verdict(r3), r3.pitchPeak, r3.tRecover);
plotRun(r3, 'Test 3: 0.10 N.s forward push at t = 1 s', fullfile(resDir,'test3_push.png'));

%% 4. Push limit, 0.5 N steps, forward and backward
for dirn = [1 -1]
    F = 0;  last = 0;
    while true
        F = F + 0.5;
        r = runOne(mdl, 1, dirn*F, 5);
        if r.fell || F >= 12, break; end
        last = F;
    end
    pred = J_max_fwd;  name = 'forward';
    if dirn < 0, pred = J_max_bwd;  name = 'backward'; end
    logf('4. Push limit %-8s      : survives %.1f N, falls at %.1f N -> %.3f..%.3f N.s (hand calc %.3f N.s)\n', ...
         name, last, F, last*0.05, F*0.05, pred);
end

%% 5. Delay sensitivity: 1, 2, 3 extra controller samples (10 ms each)
for n = 1:3
    set_param(dly, 'Commented','off', 'DelayLength', num2str(n), 'InitialCondition', sprintf('repmat(HOME,1,%d)', n));
    r = runOne(mdl, 1, 2, 6);
    logf('5. +%d sample delay, 2 N push  : %s | peak pitch %.1f deg, band %.2f deg\n', n, verdict(r), r.pitchPeak, r.pitchBand);
end
set_param(dly, 'Commented','through');

%% 6. Mass error: torso +10 %% / -10 %%
for tag = ["mplus","mminus"]
    clear mj_initbus          % fresh helper process: the blockset's bus parser can crash on repeated model loads
    set_param(plant, 'xmlFileRel', fullfile(DUCK_PROJ,'model', "duck_v3_" + tag + "_scene.xml"));
    ra = runOne(mdl, 1, 0, 20);  rb = runOne(mdl, 1, 2, 6);
    logf('6. Torso mass %-6s         : stand %s (band %.2f deg), 2 N push %s (peak %.1f deg)\n', ...
         tag, verdict(ra), ra.pitchBand, verdict(rb), rb.pitchPeak);
end
clear mj_initbus
set_param(plant, 'xmlFileRel', DUCK_XML, 'renderingType', 'Global');
fclose(fid);
fprintf('\nSaved %s\n', fullfile(resDir,'test_results.txt'));

%% ------------------------------------------------------------------ helpers
function r = runOne(mdl, balance, F, T)
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('balance_on', balance).setVariable('push_F', F);
    in = in.setModelParameter('StopTime', num2str(T));
    out = sim(in);
    q = squeeze(out.quat.Data);  if size(q,1) ~= 4, q = q.'; end
    r.t = out.quat.Time(:);
    r.pitch = rad2deg(asin(max(-1, min(1, 2*(q(1,:).*q(3,:) - q(4,:).*q(2,:))))))';   % true pitch
    z = squeeze(out.torso_pos.Data);  if size(z,1) ~= 3, z = z.'; end
    r.z = z(3,:)';
    r.te = out.pitch_est.Time(:);  r.pitchEst = rad2deg(squeeze(out.pitch_est.Data(:)));
    r.fell = any(r.z < 0.6*r.z(1));
    r.tFall = NaN;  if r.fell, r.tFall = r.t(find(r.z < 0.6*r.z(1), 1)); end
    home = evalin('base','HOME');
    knee = squeeze(out.q_left_knee.Data(:));
    r.kneeSag = abs(rad2deg(knee(find(r.t >= 0.25, 1)) - home(4)));
    ok = r.t > 2;  if r.fell, ok = r.t < r.tFall; end
    r.pitchMean = mean(r.pitch(ok));
    r.pitchBand = max(abs(r.pitch(ok) - r.pitchMean));
    post = r.t >= 1;
    r.pitchPeak = max(abs(r.pitch(post) - r.pitch(find(r.t >= 0.99, 1))));
    settle = r.pitch(end);
    bad = find(post & abs(r.pitch - settle) > 1, 1, 'last');
    r.tRecover = 0;  if ~isempty(bad), r.tRecover = r.t(bad) - 1; end
end

function logBoth(fid, varargin)
    fprintf(varargin{:});  fprintf(fid, varargin{:});
end

function s = verdict(r)
    if r.fell, s = 'FELL'; else, s = 'stands'; end
end

function plotRun(r, ttl, file)
    f = figure('Visible','off', 'Position',[100 100 800 380]);
    plot(r.t, r.pitch, 'LineWidth',1.3); hold on;  stairs(r.te, r.pitchEst, '--');
    grid on;  xlabel('time (s)');  ylabel('torso pitch (deg)');
    legend('true pitch (MuJoCo)', 'complementary-filter estimate', 'Location','best');  title(ttl);
    exportgraphics(f, file, 'Resolution',130);  close(f);
end
