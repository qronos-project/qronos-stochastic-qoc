function write_string_to_file(filename, string)
% write_string_to_file(filename, string):
% write string into the given file. old content will be overwritten.
f = fopen(filename, 'w');
assert(f > 0)
fwrite(f, string);
fclose(f);
end

