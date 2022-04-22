function File_lines =  Fcn_read_txt_lines(filename)

fid = fopen(filename);
File_lines = 0;
while ~feof(fid)
    fgetl(fid);
    File_lines = File_lines +1;
end
fclose(fid);

end