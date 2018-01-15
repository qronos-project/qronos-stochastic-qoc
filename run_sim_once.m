function [output] = run_sim_once(model, param)
% Run simulation, extract output values
% model: simulink model filename (without .slx suffix)
% param: struct with parameters (currently only used for the following
%        one parameter)
% param.SimulationMode: Simulink Accelerator mode, see
%   https://de.mathworks.com/help/simulink/ug/interacting-with-the-acceleration-modes-programmatically.html?requestedDomain=www.mathworks.com
%   Permitted values are:  'normal', 'accelerator', 'rapid'.

load_system(model)

% The following could be enabled later to pass the whole param struct to
% Simulink, if passing arguments by workspace is no longer desired:
%
% simulink_workspace = get_param(model,'modelworkspace');
% simulink_workspace.assignin('p', param)

output = struct();
output.in = param;
output.out = Simulink.SimulationOutput();
output.error = '';
output.J = [];

% Simulation accelerator mode: default to rapid accelerator
accelMode = 'rapid';
if isfield(param, 'SimulationMode')
    accelMode = getfield(param, 'SimulationMode');
end

try
    output.out = sim(model, 'ReturnWorkspaceOutputs', 'on', 'SimulationMode', accelMode);
    output.J = output.out.get('J');
catch exception
    disp('Cannot simulate for parameters:')
    disp(param)
    exceptionId = exception.identifier;
    if strcmp(exceptionId, 'SimulinkExecution:Equation:SolverError') && length(exception.cause) == 1
        % unpack cause of SolverError for accelerator mode
        exceptionId = exception.cause{1}.identifier;
    end
    if strcmp(exceptionId, 'Simulink:tools:rapidAccelExeError')
        % unpack cause of error for rapid accelerator mode (Matlab R2015a)
        if ~isempty(regexp(exception.message, 'Continuous state derivative of ''.*'' is non-finite', 'once'))
            exceptionId = 'SimulinkSolver:ODE:StateNotFinite';
        end
    end
    if ~strcmp(exceptionId, 'Simulink:Engine:DerivNotFinite') ...
            && ~strcmp(exceptionId, 'SimulinkSolver:ODE:StateNotFinite') ...
            && ~strcmp(exceptionId, 'SimulinkExecution:DE:DerivativeIsNonFinite')
        disp(['-> unknown exception ', exceptionId, '. Stopping.'])
        exception.rethrow
    end
    
    output.error = 'Integrator overflow. Replacing result by Inf.';
    disp(output.error)
    output.J = Inf;
end
end

