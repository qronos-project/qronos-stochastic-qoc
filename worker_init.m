% Initialize environment for example scripts.
% Example scripts start with worker_init and end with worker_finalize.

clear
time_worker_started = tic;
param_global

if ~isdir('plot')
    mkdir('plot')
end

% make sure all Simulink diagrams are saved
% and close them (forces reload if a Simulink file was changed by 'git pull')
while ~strcmp(gcs, '')
    close_system
end
bdclose('all') % probably not necessary

% load matlab2tikz if not yet loaded
if strcmp(which('matlab2tikz'), '')
    addpath('./matlab2tikz/src')
end

% load matlab-xunit if not yet loaded
if strcmp(which('assertElementsAlmostEqual'), '')
    addpath('./matlab-xunit/src')
end

