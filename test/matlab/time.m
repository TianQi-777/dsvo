close all
clear
dir = '~/.ros';
tt = load(strcat(dir,'/time.txt'));
t_f = tt(tt(:,1)==0,2:3);  % whole frame
t_stereo = tt(tt(:,1)==3,2:3);  % stereo match
t_normal = tt(tt(:,1)==1,2:3);  % normal frame
t_dsvo = tt(tt(:,1)==2,2:3);  % keyframe
t_feature = tt(tt(:,1)==11,2:3);  % feature tracking
t_prap_dir = tt(tt(:,1)==121,2:3);  % pose propagation, direct
t_prap_rfof = tt(tt(:,1)==122,2:3);  % pose propagation, refine by OF
t_pts = tt(tt(:,1)==131,2:3);  % points reconstruction 
t_scale = tt(tt(:,1)==132,2:3);  % scale opt time
pts = tt(tt(:,4)>=0, [2,4]);

fprintf('\nAve. time for [%d] stereo match = %f\n', size(t_stereo,1), mean(t_stereo(:,2)));
fprintf('Ave. time for [%d] normal frame = %f\n', size(t_normal,1), mean(t_normal(:,2)));
fprintf('Ave. time for [%d] DSVO Keyframe = %f\n', size(t_dsvo,1), mean(t_dsvo(:,2)));
fprintf('Ave. time for [%d] frame = %f\n', size(t_f,1), mean(t_f(:,2)));
fprintf('Ave. feature tracking time = %f\n', mean(t_feature(:,2)));
fprintf('Ave. pose propagation(direct) time = %f\n', mean(t_prap_dir(:,2)));
fprintf('Ave. pose propagation(refine by OF) time = %f\n', mean(t_prap_rfof(:,2)));
fprintf('Ave. points reconstruction time = %f\n', mean(t_pts(:,2)));
fprintf('Ave. scale optimization time = %f\n', mean(t_scale(:,2)));
fprintf('Ave. # of points = %f\n', mean(pts(:,2)));

% figure('Name','Points')
% plot(pts(:,1), pts(:,2), 'b-');
% figure('Name','VO running time')
% plot(t_normal(:,1), t_normal(:,2), 'g*-');
% hold on
% plot(t_dsvo(:,1), t_dsvo(:,2), 'b*-');
% hold on
% plot(t_scale(:,1), t_scale(:,2), 'r*-');
% ylabel('runing time / ms');
% legend('Normal frame', 'Keyframe', 'Scale optimization time');