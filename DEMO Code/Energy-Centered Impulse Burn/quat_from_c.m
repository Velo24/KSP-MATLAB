function q = quat_from_c(C, put_scalar_last)
%Return quaternion corresponding same sense as C

%Calculate quaternion from rotation matrix (This follows the Hughes
%definition i.e., q == q_si (rotation into sensor frame) 
%We assume that q(1) is the scalar and q(2:4) is the vector part

% (c) March 2011 John Enright
% http://sail.blog.ryerson.ca/
% This work is licensed under a Creative Commons Attribution 4.0
% International License

if nargin < 2
    put_scalar_last = 0;
end

q = zeros(1,4);

%Take positive root (forces unique eta)
tmp = (1+C(1,1)+C(2,2)+C(3,3));
if tmp < 0
    tmp =0;
end
q(1) = .5*sqrt(tmp);

if (q(1) > eps)
    %Indices are reversed from Hughes' definition since the C we generate
    %is C_is not C_si
    q(2:4) = (.25/q(1))*[(C(2,3) - C(3,2)), (C(3,1) - C(1,3)), (C(1,2) - C(2,1))];
else
    %Work out the special cast
    q(2:4) = sqrt(.5*(1+[C(1,1), C(2,2), C(3,3)]));
    %assume that q(1) is positive
    sa12 = sign(C(1,2));
    sa23 = sign(C(2,3));
    sa31 = sign(C(3,1));
    if (sa12 < 0)
        q(3) = -q(3);
    end
    if (sa31 < 0)
        q(4) = - q(4);
    end
    
end


%normalize the quaternion, just to be sure.
q = q/norm(q);

if put_scalar_last
   q = q([2:4 1]); 
end
