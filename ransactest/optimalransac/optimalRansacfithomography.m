% OPTIMALRANSACFITHOMOGRAPHY - fits 2D homography using OPTIMALRANSAC
%
% Usage:   
% [H, inliers, iter] = optimalRansacfithomography(x1, x2, t, acc, maxit, nrtrials, low, ner)
%
% Arguments:
%     x1        - 2xN or 3xN set of homogeneous points.  If the data is
%                 2xN it is assumed the homogeneous scale factor is 1.
%     x2        - 2xN or 3xN set of homogeneous points such that x1<->x2.
%
%
%     s         - The minimum number of samples from x required by
%                 fittingfn to fit a model.
%
%     t         - The distance threshold between data point and the model
%                 used to decide whether a point is an inlier or not. 
%                 Note that point coordinates are normalised to that their
%                 mean distance from the origin is sqrt(2).  The value of
%                 t should be set relative to this, say in the range 
%                 0.001 - 0.01  
%     acc       - The same as above but is used to prune the set. If acc<t
%                 then pruning is performed.
%     maxit     - Do not repeat rescoring more than maxit times. 
%                 Rescoring is performed until the set does not change
%                 anymore but if the set wobbles maxit prevents it from
%                 getting stuck.
%     nrtrials  - Defines how many resamplings shall be done.
%     low       - How many inliers are required before optimization is done.
%     ner       - Number of equal sets required before stopping.
%                 1 means that one pair of equal sets have been obtaines.
%                 2 Means that three sets were equal etc.
%
% Returns:
%     H         - The 3x3 homography such that x2 = H*x1.
%     inliers   - An array of indices of the elements of x1, x2 that were
%                 the inliers for the best model.
%
% See Also: ransac, homography2d, homography1d, optimalRansac, resample, rescore
% 
%
% References:
%    M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
%    for model fitting with applications to image analysis and automated
%    cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981
%
%    Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
%    Computer Vision". pp 101-113. Cambridge University Press, 2001
% 
% Originaly rewritten from and to be used with code that is Copyright (c) 2003-2006 Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% pk at csse uwa edu au    
% http://www.csse.uwa.edu.au/~pk
%
% Copyright (c) 2011-2013 Anders Hast
% Uppsala University
% http://www.cb.uu.se/~aht
% 
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software.
%
% The Software is provided "as is", without warranty of any kind.
%
% History
% 8/12-2012 AHT. homogdist2d is changed to return a zero matrix instead of
% throwing an error when the set is degenerate. Hence, it is not necessary
% to check it as some sets are quasi degenerate.
%
% AHT 10/6 2015. Changed computation of iterations
%
function [H, inliers] = optimalRansacfithomography(x1, x2, t, acc, maxit, nrtrials, low, ner)

    narginchk ( 4, 8 );
    nargoutchk ( 2, 3 );

    if nargin == 4
        maxit=20;
        nrtrials=8;
        low=5;
        ner=1;
    end

    if ~all(size(x1)==size(x2))
        error('Data sets x1 and x2 must have the same dimension');
    end
    
    [rows,npts] = size(x1);
    if rows~=2 & rows~=3
        error('x1 and x2 must have 2 or 3 rows');
    end
    
    if npts < 4
        error('Must have at least 4 points to fit homography');
    end

    if rows == 2    % Pad data with homogeneous scale factor of 1
        x1 = [x1; ones(1,npts)];
        x2 = [x2; ones(1,npts)];        
    end
        
    % Normalise each set of points so that the origin is at centroid and
    % mean distance from origin is sqrt(2).  normalise2dpts also ensures the
    % scale parameter is 1.  Note that 'homography2d' will also call
    % 'normalise2dpts' but the code in 'ransac' that calls the distance
    % function will not - so it is best that we normalise beforehand.
    [x1, T1] = normalise2dpts(x1);
    [x2, T2] = normalise2dpts(x2);
    
    s = 4;  % Minimum No of points needed to fit a homography.
    
    fittingfn = @homography2d;
    distfn    = @homogdist2d;
    % x1 and x2 are 'stacked' to create a 6xN array for ransac
    [H, inliers] = optimalRansac([x1; x2], fittingfn, distfn, s, t, acc, 100, 4000, maxit, nrtrials, low, ner);
    
    % Denormalise
    H = T2\H*T1;
end
%----------------------------------------------------------------------
% Function to evaluate the symmetric transfer error of a homography with
% respect to a set of matched points as needed by RANSAC.
% AHT 20/8-2012. Removed the output parameter H as it is not computed and
% therefore confuses the user.
% AHT 30/1-2013. Return d2 for use in PRUNESET

function [inliers, d2] = homogdist2d(H, x, t);
    
    x1 = x(1:3,:);   % Extract x1 and x2 from x
    x2 = x(4:6,:);    
    
    % Calculate, in both directions, the transfered points    
    Hx1    = H*x1;
    invHx2 = H\x2;
    
    % Normalise so that the homogeneous scale parameter for all coordinates
    % is 1.
    
    x1     = hnormalise(x1);
    x2     = hnormalise(x2);     
    Hx1    = hnormalise(Hx1);
    invHx2 = hnormalise(invHx2); 
    
    d2 = sum((x1-invHx2).^2)  + sum((x2-Hx1).^2);
    inliers = find(abs(d2) < t);    
    
end
