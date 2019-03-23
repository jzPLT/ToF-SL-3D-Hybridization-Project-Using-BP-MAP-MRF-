function H=lpfilter(type,M,N,D0,n)

[U,V] = dftuv(M,N);
D = hypot(U,V);

    if D0 >0
        switch type
            case 'ideal'
                if nargin>=4
                    H = single(D <= D0);
              
                end
            case 'btw'
                if nargin == 4
                    n = 1.0;
                end
                H = 1./ (1+ ((D./D0).^(2*n)));

            case 'gaussian'
                H = exp(-D.^2/(2*(D0^2)));
        end
    else
        H = ones(M,N);
    end

H = fftshift(H);  
    
end


