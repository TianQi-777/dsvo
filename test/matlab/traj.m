close all; 
vo = load('~/.ros/vo.txt'); 
vo = alignZ(vo(:,3:5));
plot3(vo(:,1), vo(:,2), vo(:,3), 'r.'); 
axis equal