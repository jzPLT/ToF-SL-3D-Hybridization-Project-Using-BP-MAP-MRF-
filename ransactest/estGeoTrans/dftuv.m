function [U,V] = dftuv(M,N)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    x = 0:M-1;
    y = 0:N-1;
    
    idx = find(x > M/2);
    x(idx) = x(idx) - M;
    
    idx = find(y > N/2);
    y(idx) = y(idx) - N;
    
    [V,U] = meshgrid(y,x);
end

