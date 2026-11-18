%% run_tests.m -- PDF Part 8 tests, run on duck_sim.slx (Simulink controller + MuJoCo plant), with pass/fail assertions
% Usage:  startup_duck; run_tests            (needs duck_sim.slx; build_duck_sim makes it)
% Results print to the command window and go to results/post_fix/test_results.txt + PNG plots.
% EXIT STATUS: under `matlab -batch run_tests` this script raises an error (nonzero exit) when any assertion fails.
% Results of the earlier model (before black PLA / bill / collision / mass changes) are kept in results/pre_fix/.
%
% Acceptance criteria (declared up front, same numbers as scripts/motions.py balance where they overlap):
CRIT.maxTiltDeg      = 10;     % torso tilt from vertical (pitch AND roll), any sample
CRIT.minHeightFrac   = 0.87;   % torso height >= this fraction of the initial height, any sample
CRIT.maxDriftM       = 0.10;   % horizontal drift of the torso over the run
CRIT.settleBandDeg   = 1.0;    % pitch must stay within this band around its final value ...
CRIT.settleByS       = 5.0;    % ... from this time on (steady-state runs)
CRIT.minJointMarginR = 0.02;   % logged knee/ankle joint at least this far from its +-1.5708 rad limit
CRIT.maxSatRunS      = 1.0;    % longest continuous ankle-command saturation (|offset| = SAT_ANKLE)
CRIT.recoverByS      = 2.0;    % small push: pitch back within 1 deg by this many seconds after the push
CRIT.pushCapN        = 12;     % push-limit SEARCH CAP: reaching it means "bounded below", not "falls at"
CRIT.minLimitFrac    = 0.5;    % measured survived impulse >= this fraction of the hand-calculation (sanity)
if ~exist('DUCK_PROJ','var'), startup_duck; end
load(fullfile(DUCK_PROJ,'matlab','duck_params.mat'));
mdl = 'duck_sim';  load_system(mdl);
plant = [mdl '/MuJoCo Plant'];  dly = [mdl '/Extra delay'];
set_param(plant, 'renderingType', 'None');            % no window: tests run much faster
resDir = fullfile(DUCK_PROJ,'results','post_fix');  if ~isfolder(resDir), mkdir(resDir); end
fid = fopen(fullfile(resDir,'test_results.txt'),'w');
logf = @(varargin) logBoth(fid, varargin{:});
nFail = 0;  failed = {};
logf('Duckling V3 Simulink+MuJoCo tests (POST-FIX model), %s\n', datestr(now));
logf('model: %s\n', fileread_first(fullfile(DUCK_PROJ,'model','stand_numbers.json'), 'model_scene_sha256'));
logf('gains: ankle = %.2f*pitch + %.3f*rate\n', K_TH, K_D);
logf('criteria: tilt<=%g deg, height>=%.0f%%, drift<=%g m, settle band %g deg by %g s, joint margin>=%g rad, saturation run<=%g s\n\n', ...
     CRIT.maxTiltDeg, 100*CRIT.minHeightFrac, CRIT.maxDriftM, CRIT.settleBandDeg, CRIT.settleByS, CRIT.minJointMarginR, CRIT.maxSatRunS);

%% 1. Hold pose, balance loop off: the open loop is unstable, so a fall is the EXPECTED result
r = runOne(mdl, 0, 0, 10, CRIT, HOME, SAT_ANKLE);
logf('1. Hold pose, loop OFF, 10 s : %s', verdict(r));
if r.fell, logf(' (tips over at t = %.2f s)', r.tFall); end
logf('\n   knee sag at t = 0.25 s: %.1f deg (hand calc %.1f deg)\n', r.kneeSag, sag_knee_deg);
[nFail, failed] = assertOK(logf, nFail, failed, '1 open loop tips over (expected)', r.fell);

%% 2. Stand, loop on, 60 s
r2 = runOne(mdl, 1, 0, 60, CRIT, HOME, SAT_ANKLE);
logf('2. Stand, loop ON, 60 s      : %s | pitch mean %+.2f deg, band +/- %.2f deg around it (after 2 s)\n', ...
     verdict(r2), r2.pitchMean, r2.pitchBand);
logf('   tilt max %.2f deg | min height %.0f mm (start %.0f) | drift %.3f m | knee/ankle margin %.3f rad | longest saturation %.2f s | settled by %.1f s\n', ...
     r2.tiltMax, 1000*r2.zMin, 1000*r2.z0, r2.drift, r2.jointMargin, r2.satRunS, r2.tSettle);
plotRun(r2, 'Test 2: stand, balance loop on', fullfile(resDir,'test2_stand.png'));
[nFail, failed] = assertOK(logf, nFail, failed, '2a no fall', ~r2.fell);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2b tilt max %.2f <= %g deg', r2.tiltMax, CRIT.maxTiltDeg), r2.tiltMax <= CRIT.maxTiltDeg);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2c min height %.0f mm >= %.0f mm', 1000*r2.zMin, 1000*CRIT.minHeightFrac*r2.z0), r2.zMin >= CRIT.minHeightFrac*r2.z0);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2d drift %.3f <= %g m', r2.drift, CRIT.maxDriftM), r2.drift <= CRIT.maxDriftM);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2e settled within %g deg by %.1f s <= %g s', CRIT.settleBandDeg, r2.tSettle, CRIT.settleByS), r2.tSettle <= CRIT.settleByS);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2f joint margin %.3f >= %g rad (knee, ankle only: the only logged joints)', r2.jointMargin, CRIT.minJointMarginR), r2.jointMargin >= CRIT.minJointMarginR);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('2g longest ankle saturation %.2f <= %g s', r2.satRunS, CRIT.maxSatRunS), r2.satRunS <= CRIT.maxSatRunS);

%% 3. Small push 2 N x 0.05 s = 0.10 N.s
r3 = runOne(mdl, 1, 2, 6, CRIT, HOME, SAT_ANKLE);
logf('3. Push 2 N x 0.05 s (0.10 N.s): %s | peak pitch %.1f deg, back within 1 deg after %.2f s, tilt max %.1f deg, saturation run %.2f s\n', ...
     verdict(r3), r3.pitchPeak, r3.tRecover, r3.tiltMax, r3.satRunS);
plotRun(r3, 'Test 3: 0.10 N.s forward push at t = 1 s', fullfile(resDir,'test3_push.png'));
[nFail, failed] = assertOK(logf, nFail, failed, '3a no fall', ~r3.fell);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('3b tilt max %.1f <= %g deg', r3.tiltMax, CRIT.maxTiltDeg), r3.tiltMax <= CRIT.maxTiltDeg);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('3c recovered within 1 deg after %.2f s <= %g s', r3.tRecover, CRIT.recoverByS), r3.tRecover <= CRIT.recoverByS);
[nFail, failed] = assertOK(logf, nFail, failed, sprintf('3d saturation run %.2f <= %g s', r3.satRunS, CRIT.maxSatRunS), r3.satRunS <= CRIT.maxSatRunS);

%% 4. Push limit, 0.5 N steps, forward and backward (search CAP = CRIT.pushCapN)
for dirn = [1 -1]
    F = 0;  last = 0;  hitCap = false;
    while true
        F = F + 0.5;
        r = runOne(mdl, 1, dirn*F, 5, CRIT, HOME, SAT_ANKLE);
        if r.fell, break; end
        last = F;
        if F >= CRIT.pushCapN, hitCap = true; break; end
    end
    pred = J_max_fwd;  name = 'forward';
    if dirn < 0, pred = J_max_bwd;  name = 'backward'; end
    if hitCap
        logf('4. Push limit %-8s      : NO FALL up to the search CAP of %g N -> BOUND only: survives >= %.3f N.s (hand calc %.3f N.s); not a failure level\n', ...
             name, CRIT.pushCapN, last*0.05, pred);
    else
        logf('4. Push limit %-8s      : survives %.1f N, falls at %.1f N -> %.3f..%.3f N.s (hand calc %.3f N.s)\n', ...
             name, last, F, last*0.05, F*0.05, pred);
    end
    [nFail, failed] = assertOK(logf, nFail, failed, sprintf('4 %s survived impulse %.3f >= %.0f%% of hand calc %.3f N.s', name, last*0.05, 100*CRIT.minLimitFrac, pred), last*0.05 >= CRIT.minLimitFrac*pred);
end

%% 5. Delay sensitivity: 1, 2, 3 extra controller samples (10 ms each)
for n = 1:3
    set_param(dly, 'Commented','off', 'DelayLength', num2str(n), 'InitialCondition', sprintf('repmat(HOME,1,%d)', n));
    r = runOne(mdl, 1, 2, 6, CRIT, HOME, SAT_ANKLE);
    logf('5. +%d sample delay, 2 N push  : %s | peak pitch %.1f deg, band %.2f deg, tilt max %.1f deg\n', n, verdict(r), r.pitchPeak, r.pitchBand, r.tiltMax);
    [nFail, failed] = assertOK(logf, nFail, failed, sprintf('5 +%d sample delay: no fall, tilt %.1f <= %g deg', n, r.tiltMax, CRIT.maxTiltDeg), ~r.fell && r.tiltMax <= CRIT.maxTiltDeg);
end
set_param(dly, 'Commented','through');

%% 6. Mass error: torso +10 %% / -10 %%
for tag = ["mplus","mminus"]
    clear mj_initbus          % fresh helper process: the blockset's bus parser can crash on repeated model loads
    set_param(plant, 'xmlFileRel', fullfile(DUCK_PROJ,'model', "duck_v3_" + tag + "_scene.xml"));
    ra = runOne(mdl, 1, 0, 20, CRIT, HOME, SAT_ANKLE);  rb = runOne(mdl, 1, 2, 6, CRIT, HOME, SAT_ANKLE);
    logf('6. Torso mass %-6s         : stand %s (band %.2f deg, tilt max %.1f), 2 N push %s (peak %.1f deg, tilt max %.1f)\n', ...
         tag, verdict(ra), ra.pitchBand, ra.tiltMax, verdict(rb), rb.pitchPeak, rb.tiltMax);
    [nFail, failed] = assertOK(logf, nFail, failed, sprintf('6 %s: stands and survives the 2 N push', tag), ...
        ~ra.fell && ~rb.fell && ra.tiltMax <= CRIT.maxTiltDeg && rb.tiltMax <= CRIT.maxTiltDeg);
end
clear mj_initbus
set_param(plant, 'xmlFileRel', DUCK_XML, 'renderingType', 'Global');
logf('\n%d assertion(s) FAILED%s\n', nFail, ternary(nFail>0, [': ' strjoin(failed, ' | ')], ''));
fclose(fid);
fprintf('\nSaved %s\n', fullfile(resDir,'test_results.txt'));
if nFail > 0
    error('run_tests:failed', '%d regression assertion(s) failed: %s', nFail, strjoin(failed, ' | '));
end

%% ------------------------------------------------------------------ helpers
function r = runOne(mdl, balance, F, T, CRIT, HOME, SAT)
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('balance_on', balance).setVariable('push_F', F);
    in = in.setModelParameter('StopTime', num2str(T));
    out = sim(in);
    q = squeeze(out.quat.Data);  if size(q,1) ~= 4, q = q.'; end
    r.t = out.quat.Time(:);
    r.pitch = rad2deg(asin(max(-1, min(1, 2*(q(1,:).*q(3,:) - q(4,:).*q(2,:))))))';   % true pitch
    r.tilt  = rad2deg(acos(max(-1, min(1, 1 - 2*(q(2,:).^2 + q(3,:).^2)))))';          % angle of the torso z axis from vertical
    r.tiltMax = max(r.tilt(r.t >= 0.1));
    z = squeeze(out.torso_pos.Data);  if size(z,1) ~= 3, z = z.'; end
    r.z = z(3,:)';
    % the plant logs an all-zero first sample, so every height/drift reference uses the first sample at t >= 0.1 s
    k0 = find(r.t >= 0.1, 1);
    r.z0 = r.z(k0);
    r.zMin = min(r.z(k0:end));
    r.drift = max(vecnorm(z(1:2,k0:end) - z(1:2,k0), 2, 1));
    r.te = out.pitch_est.Time(:);  r.pitchEst = rad2deg(squeeze(out.pitch_est.Data(:)));
    % continuous fall monitoring: height OR tilt at any sample
    fellMask = (r.t >= 0.1) & ((r.z < 0.6*r.z0) | (r.tilt > 45));
    r.fell = any(fellMask);
    r.tFall = NaN;  if r.fell, r.tFall = r.t(find(fellMask, 1)); end
    knee = squeeze(out.q_left_knee.Data(:));  ankle = squeeze(out.q_left_ankle.Data(:));
    r.kneeSag = abs(rad2deg(knee(find(r.t >= 0.25, 1)) - HOME(4)));
    r.jointMargin = 1.5708 - max(max(abs(knee(k0:end))), max(abs(ankle(k0:end))));
    ok = r.t > 2;  if r.fell, ok = r.t < r.tFall; end
    r.pitchMean = mean(r.pitch(ok));
    r.pitchBand = max(abs(r.pitch(ok) - r.pitchMean));
    post = r.t >= 1;
    r.pitchPeak = max(abs(r.pitch(post) - r.pitch(find(r.t >= 0.99, 1))));
    settle = r.pitch(end);
    bad = find(post & abs(r.pitch - settle) > 1, 1, 'last');
    r.tRecover = 0;  if ~isempty(bad), r.tRecover = r.t(bad) - 1; end
    badS = find(abs(r.pitch - settle) > CRIT.settleBandDeg, 1, 'last');
    r.tSettle = 0;  if ~isempty(badS), r.tSettle = r.t(badS); end
    % actuator saturation: commanded left-ankle offset from HOME reaching the +-SAT_ANKLE clip (u logs the 16 plant inputs)
    U = squeeze(out.u.Data);  if size(U,1) ~= 16, U = U.'; end
    sat = abs(U(5,:) - HOME(5)) >= SAT - 1e-4;
    dtu = median(diff(out.u.Time(:)));
    run = 0; best = 0;
    for k = 1:numel(sat), if sat(k), run = run + 1; best = max(best, run); else, run = 0; end, end
    r.satRunS = best * dtu;
end

function [nFail, failed] = assertOK(logf, nFail, failed, name, ok)
    if ok, logf('   [PASS] %s\n', name); else, logf('   [FAIL] %s\n', name); nFail = nFail + 1; failed{end+1} = name; end
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end

function s = fileread_first(file, key)
    t = jsondecode(fileread(file));
    if isfield(t, key), s = t.(key); else, s = 'unknown (stand_numbers.json has no provenance field)'; end
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
