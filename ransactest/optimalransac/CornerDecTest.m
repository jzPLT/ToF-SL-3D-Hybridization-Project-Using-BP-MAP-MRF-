clear all
close all
% clc
% I = im2double(imread('lena.tif'));
I = checkerboard(10);
theta = 45;
shearFacY = 1/sqrt(2);
shearFacX = 0;


theta = 10;
tm = [cosd(theta) -sind(theta) 0; ...
    sind(theta) cosd(theta) 0; ...
    0 0 1];
% tm = [1 5 0;2 1 0;0 0 1];
% tm = [1 0 0;0 1 0;15 -7 1];
mt = inv(tm)
tform = projective2d(tm);
Iwarp = imwarp(I,tform);

points = detectMinEigenFeatures(I);
warpPoints = detectMinEigenFeatures(Iwarp);
strongPoints = points.selectStrongest(49);
strongWarp = warpPoints.selectStrongest(49);
strongPoints.Location = round(strongPoints.Location);
strongWarp.Location = round(strongWarp.Location);

figure(1)
subplot(1,2,1)
imshow(Iwarp);hold on;
plot(strongWarp);
subplot(1,2,2)
imshow(I);hold on;
plot(strongPoints);

[H,~] = optimalRansacfithomography(transpose(strongPoints.Location),...
    transpose(strongWarp.Location),0.01,0.01)
% H(:,3) = [0.6;-0.3;1]
% H(3,:) = [0,0,1];
A = H';
tformH = projective2d(A);
% tformH = fitgeotrans(strongWarp.Location,strongPoints.Location,'projective');
out = imwarp(Iwarp,tformH);

% [M,N]= size(I)
% oriIwarp = zeros(M,N);
% for i = 1:M
%     for j = 1:N
%         B{i,j} = A*[i;j;1];
%         
%     end
% end

% oriIwarp = imwarp(Iwarp,A);

figure(2)
imshow(out);



