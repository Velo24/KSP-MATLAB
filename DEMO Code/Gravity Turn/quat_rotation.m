function R = quat_rotation(q,scalar_last)
% Return the 3x3 rotation matrix, R, corresponding to quaternion, q.
% Default quaternion is [qs qv] (i.e. scalar then vector.)
% If you want to use [qv qs] then set scalar_last equal to 1
% q is assumed to have the sense q_21 -- from 1 to 2. Thus the rotation we
% return is in the same sense.
%
% (c) March 2011 John Enright
% http://sail.blog.ryerson.ca/
% This work is licensed under a Creative Commons Attribution 4.0
% International License

if nargin < 2
   scalar_last = 0; 
end

if scalar_last
    %permute vector
   q = q([4 1 2 3]); 
end

q_sq = q.^2;
R = [(q_sq(1) + q_sq(2) - q_sq(3) - q_sq(4)) 2*(q(2)*q(3) - q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));
    2*(q(2)*q(3) + q(1)*q(4)) (q_sq(1) + q_sq(3) - q_sq(2) - q_sq(4)) 2*(q(3)*q(4)-q(1)*q(2));
    2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) (q_sq(1) + q_sq(4) - q_sq(2) - q_sq(3))].';
