clear 
close all

dir = '/home/jiawei/Desktop/results/grass';
% dir = '~/.ros';
stereo = load(strcat(dir,'/stereo.txt'));
dsvo = load(strcat(dir,'/vo.txt'));
dense = load(strcat(dir,'/truth.txt'));

stereo = alignZ(stereo(:,3:5));
dsvo = alignZ(dsvo(:,3:5));
dense = alignZ(dense(:,2:4));
figure('Name','Stereo')
plot3(stereo(:,1), stereo(:,2), stereo(:,3), 'g.')
axis equal
figure('Name','DSVO')
plot3(dsvo(:,1), dsvo(:,2), dsvo(:,3), 'r.')
axis equal
figure('Name','ZED')
plot3(dense(:,1), dense(:,2), dense(:,3), 'b.')
axis equal