close all; 
vo = load('~/.ros/vo.txt'); plot3(vo(:,3), vo(:,4), vo(:,5), 'r-'); hold on
rg = 1:4000;
gt = load('/home/jiawei/Documents/direct_stereo_data/kitti/dataset/poses/00.txt'); plot3(gt(rg,4), gt(rg,8), gt(rg,12), 'g-'); 
axis equal