%% startup_duck.m -- run this first in every MATLAB session
% Adds the MuJoCo blockset and this project's scripts to the path.
proj = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(proj,'matlab'));
addpath(fullfile(proj,'mujoco-simulink-blockset','blocks'));
addpath(fullfile(proj,'mujoco-simulink-blockset','examples'));
addpath(fullfile(proj,'mujoco-simulink-blockset','src'));
setenv('PATH', [fullfile(proj,'mujoco-simulink-blockset','blocks') ';' getenv('PATH')]);  % mujoco.dll, glfw3.dll
DUCK_PROJ = proj;
DUCK_XML  = fullfile(proj,'model','duck_v4_scene.xml');
fprintf('Duckling project: %s\n', proj);
