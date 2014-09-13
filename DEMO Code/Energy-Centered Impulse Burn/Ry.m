function y=Ry(theta,reverse_sense)
% (c) March 2011 John Enright
% http://sail.blog.ryerson.ca/
% This work is licensed under a Creative Commons Attribution 4.0
% International License

if nargin < 2
    reverse_sense = 0;
end
if reverse_sense
    theta = -theta;
end
s = sin(theta);
c = cos(theta);
y=[c 0 s;0 1 0;-s 0 c];