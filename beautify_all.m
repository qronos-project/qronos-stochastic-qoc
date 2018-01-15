%% Pre-commit script which formats all code nicely
% The working directory must be the git repository in which this file lives
addpath('./MBeautifier')
MBeautify.formatFiles(pwd(), '*.m')