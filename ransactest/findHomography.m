function H = findHomography(pts1,pts2)
% [H corrPtIdx] = findHomography(pts1,pts2)
%	Find the homography between two planes using a set of corresponding
%	points. PTS1 = [x1,x2,...;y1,y2,...]. RANSAC method is used.
%	corrPtIdx is the indices of inliers.
%	Yan Ke @ THUEE, 20110123, xjed09@gmail.com


coef.minPtNum = 4;
coef.iterNum = 10000;
coef.thDist = 5;
coef.thInlrRatio = 0;
% H = ransac1(pts1,pts2,coef,@solveHomo,@calcDist);
[H,~] = optimalRansac([pts1;pts2],@solveHomo,@calcDist,coef.minPtNum,coef.thDist,coef.thDist-0.1,1)

end

function d = calcDist(H,pts1,pts2)
%	Project PTS1 to PTS3 using H, then calcultate the distances between
%	PTS2 and PTS3

n = size(pts1,2);
pts3 = H*[pts1;ones(1,n)];
pts3 = pts3(1:2,:)./repmat(pts3(3,:),2,1);
% if size(pts2,2)<n
%     pts2 = [pts2,ones(2,n-size(pts2,2))];
% end
d = sum((pts2-pts3).^2,1);

end