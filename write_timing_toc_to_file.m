function write_timing_toc_to_file(filename, info_text)
% write_timing_toc_to_file(filename
% write_timing_toc_to_file(filename, info_text)
% Call toc() and write the output to the given filename.
if nargin == 1
    info_text = 'Elapsed time is ';
end
seconds = toc();
string = [info_text, sprintf(' %d seconds', seconds)];
disp(string);
write_string_to_file(filename, string);
end

