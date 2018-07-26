clear 
close all

% dir = '~/Dropbox/results/grass_circle';
dir = '~/Dropbox/results/grass_square';
dir = '~/.ros';

figure
dsvo = load(strcat(dir,'/vo.txt'));
dsvo = alignZ(dsvo(:,3:5));
subplot(2,3,1)
plot3(dsvo(:,1), dsvo(:,2), dsvo(:,3), 'r.')
title('DSVO, XY')
view(0,90)
axis equal
subplot(2,3,4)
plot3(dsvo(:,1), dsvo(:,2), dsvo(:,3), 'r.')
title('DSVO, XZ')
view(0,0)
axis equal

zed = load(strcat(dir,'/truth.txt'));
zed = alignZ(zed(:,2:4));
subplot(2,3,2)
plot3(zed(:,1), zed(:,2), zed(:,3), 'g.')
title('Truth, XY')
view(0,90)
axis equal
subplot(2,3,5)
plot3(zed(:,1), zed(:,2), zed(:,3), 'g.')
title('Truth, XZ')
view(0,0)
axis equal

sptam = load(strcat(dir,'/sptam.txt'));
sptam = alignZ(sptam(:,2:4));
subplot(2,3,3)
plot3(sptam(:,1), sptam(:,2), sptam(:,3), 'b.')
title('SPTAM, XY')
view(0,90)
axis equal
subplot(2,3,6)
plot3(sptam(:,1), sptam(:,2), sptam(:,3), 'b.')
title('SPTAM, XZ')
view(0,0)
axis equal

function X = alignZ(XYZ)
xyz0=mean(XYZ);
A=bsxfun(@minus,XYZ,xyz0); %center the data
[~,~,V]=svd(A,0);
XYZ = A*V;
angle = XYZ(floor(size(XYZ,1)/4),1:2) - XYZ(1,1:2);
angle = atan2(angle(2), angle(1));
R = [cos(angle), sin(angle), 0; -sin(angle), cos(angle), 0; 0, 0, 1];
XYZ = XYZ * R';
X = XYZ - XYZ(1,:);
end