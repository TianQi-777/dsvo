%% load data
% close all
% clear
dir = '~/.ros';
t = load(strcat(dir,'/time.txt'));
st = load(strcat(dir,'/sptam_time.txt'));
t = [t(:,1), [1:size(t,1)]', t(:,2)];
t0 = t(find(t(:,1)==0),2:3);
t1 = t(find(t(:,1)==1),2:3);
dt = t(find(t(:,1)<2),3);
t2 = t(find(t(:,1)>1),[1,3]);

%% VO time
disp('DSVO')
% ave_t0 = sum(t0(:,2)) / size(t0,1);
% ave_t1 = sum(t1(:,2)) / size(t1,1);
% fprintf('Ave. time for [%d] normal frame = %f\nAve. time for [%d] Keyframe = %f\n', ...
% size(t0,1), ave_t0, size(t1,1), ave_t1);
fprintf('Ave. time = %f\n', sum(dt) / size(dt,1));

% figure('Name','VO running time')
% plot(t0(:,1), t0(:,2), 'g');
% hold on
% plot(t1(:,1), t1(:,2), 'b');
% ylabel('runing time / ms');
% legend('Normal frame', 'Keyframe');
% 
% %% scale time
% ave_t2 = sum(t2(:,2)) / size(t2,1);
% pts_scale = max(t2(:,2)) / max(t2(:,1));
% fprintf('Scale optimization:\nAve. point count = %f\nAve. time = %f\n', sum(t2(:,1))/size(t2(:,1),1), ave_t2);
% figure('Name','Scale optimization time')
% % plot(1:size(t2,1), pts_scale*t2(:,1), 'b-*');
% % hold on
% plot(1:size(t2,1), t2(:,2), 'r');
% ylabel('runing time / ms');


disp('SPTAM')
fprintf('Ave. time = %f\n', sum(st)/length(st));

figure('Name','Runtime comparision')
plot(dt, 'g');
hold on
plot(st, 'b');
legend('DSVO', 'SPTAM');