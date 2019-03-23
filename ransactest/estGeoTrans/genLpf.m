function [I,G,B] = genLpf(P,Q,d,n)
%This is the function for part a, where an Ideal, Gaussian, and Butterworth
%filter are return from the size and cutoff frequency of the image.

%Center the distance onto the center of the image.
for i = 1:P
    for j = 1:Q
        D(i,j)=sqrt((i-(P/2)).^2 + (j-(Q/2)).^2);
    end
end

%Ideal LPF Generation
I = double(D<=d);

%Gaussian LPF Generation
G = exp(-D.^2/(2*(d^2)));

%Butterworth LPF Generation
B = 1./ (1+ ((D/d).^(2*n)));

end


