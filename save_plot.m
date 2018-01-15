function save_plot(filename, heightFactor, width)
% save_plot(filename, heightFactor, width)
% Save plot to ./plot/<filename>.pdf, also create matlab2tikz output if it
% is available and save the workspace as well.
%
% heightFactor (optional):
%   stretch horizontally compared to the default aspect ratio.
% width (optional):
%   absolute width in centimeters.
if nargin <= 1
    heightFactor = 1;
end
if nargin <= 2
    width = 15.5;
end

if ~isdir('plot')
    mkdir('plot')
end

set(gcf, 'PaperUnits', 'centimeters')
set(gcf, 'units', 'centimeters')
size = [width, width * 12 / 15.5 * heightFactor];
set(gcf, 'position', [0, 0, size])
set(gcf, 'PaperSize', size)
set(gcf, 'PaperPositionMode', 'auto')
print(gcf, strcat('./plot/', filename, '.pdf'), '-dpdf')
savefig(strcat('./plot/', filename))
% save corresponding base workspace contents
evalin('base', ['save(''', strcat('plot/', filename), ''', ''-v7.3'')']);
if which('matlab2tikz')
    matlab2tikz(strcat('./plot/', filename, '.standalone.tex'), 'standalone', true, 'showInfo', false, 'externalData', true)
else
    warning('matlab2tikz is not available, skipping TeX output')
end
end
