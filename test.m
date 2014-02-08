filename = fullfile('baxterData.dat');
m = memmapfile(filename,'Writable',true,'Format','double');

while (1)
testing = m.Data(1:17)
end