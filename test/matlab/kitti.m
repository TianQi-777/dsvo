close all; 
vo = load('~/.ros/vo.txt'); 
vo = vo(:,3:5);
plot3(vo(:,1), vo(:,2), vo(:,3), 'r-'); hold on
gt = load('/home/jiawei/Documents/direct_stereo_data/kitti/dataset/poses/00.txt');
gt = gt(:,[4,8,12]);
plot3(gt(:,1), gt(:,2), gt(:,3), 'g-'); 
axis equal