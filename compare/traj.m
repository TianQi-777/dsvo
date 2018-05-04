close all; 
vo = load('~/.ros/vo.txt'); plot3(vo(:,2), vo(:,3), vo(:,4), 'r-'); 
view([0,1])