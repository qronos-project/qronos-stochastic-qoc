% Called at the end of every example script.
%
% Archives resulting plot/ directory into
% plot-archive/<date>-<revision>-<counter>/

toc(time_worker_started);
save('plot/workspace')
[status, gitrev] = system('git describe --always --dirty');
if (status ~= 0)
    gitrev = 'unknownGitRevision';
    warning('Cannot determine git revision.')
end
write_string_to_file('plot/matlab-version', version())
if ~isdir('plot-archive')
    mkdir('plot-archive')
end
target_dir_base = strcat('plot-archive/', datestr(datetime('now'), 'yyyy-mm-dd'), '-', gitrev);
target_dir_counter = 0;
while true
    target_dir = strcat(target_dir_base, '.', sprintf('%d', target_dir_counter));
    target_dir_counter = target_dir_counter + 1;
    if ~isdir(target_dir)
        break
    end
end
movefile('plot', target_dir)
mkdir('plot')
write_string_to_file('plot-archive/latest', target_dir);