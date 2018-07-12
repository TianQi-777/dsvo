close all; 
vo = load('~/.ros/vo.txt'); 
plot3(vo(:,3), vo(:,4), vo(:,5), 'r.'); 
axis equal