%% build_blockset.m -- download MuJoCo + GLFW and compile the Simulink Blockset for MuJoCo
% Run once:  matlab -batch "run('<repo>/matlab/build_blockset.m')"
% Replaces the repo's "install; setupBuild; build" (which needs gmake) with direct mex calls.
repo = 'C:\dev\duckling_sim\mujoco-simulink-blockset';
cd(repo);
if ~ispref('mujoco') || ~isfolder(fullfile(repo,'lib','win64','mujoco','lib'))
    install;                               % downloads MuJoCo 3.3.6 + GLFW 3.3.7 into repo\lib
end

inc  = getpref('mujoco','incPaths');
if iscell(inc), mjInc = char(inc{1}); else, mjInc = fullfile(repo,'lib','win64','mujoco','include'); end
glfwInc = char(getpref('mujoco','glfwIncPath'));
glfwLib = fullfile(char(getpref('mujoco','glfwImportLibPath')),'glfw3dll.lib');
mjLibFile   = fullfile(repo,'lib','win64','mujoco','lib','mujoco.lib');
src = fullfile(repo,'src');
out = fullfile(repo,'blocks');
clear mex
for t = ["mj_sfun","mj_initbus_mex","mj_sampletime","mj_depth_near_far"]
    fprintf('---- building %s\n', t);
    args = {mjInc, glfwInc, glfwLib, mjLibFile}; disp(args');
    mex(fullfile(src,'mj.cpp'), fullfile(src, [char(t) '.cpp']), ...
        ['-I' mjInc], ['-I' fullfile(mjInc,'mujoco')], ['-I' src], ['-I' glfwInc], ...
        'COMPFLAGS=$COMPFLAGS /EHsc /std:c++17', glfwLib, mjLibFile, ...
        '-outdir', out, '-output', char(t));
end
disp('BLOCKSET BUILD OK');
dir(fullfile(out,'*.mexw64'))
