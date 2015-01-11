function A = tread(fileID,sizeA,precision)
%{
TCP/IP READ
This fucntion reads values from the stream, reordering to match endian
used in MATLAB
%}

A = zeros(1,sizeA,precision);
for i = 1:sizeA

    switch precision
        case 'double'    
            B = fread(fileID,8,'uint8');

        case 'single'  
            B = fread(fileID,4,'uint8');            
                
        case 'int8'
            B = fread(fileID,1,'int8');
               
        case 'uint8'
            B = fread(fileID,1,'uint8');
        
        otherwise
            error('Invalid type');

    end
    A(i) = typecast(uint8(B),precision);

end
end