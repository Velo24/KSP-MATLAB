function twrite(fileID,A)
%{
TCP/IP WRITE
This fucntion writes values to the stream, reordering to match endian used
in C#
%}

sentData = typecast(A,'uint8');
fwrite(fileID,sentData,'uint8');
end