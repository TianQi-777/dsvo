%% load data
close all
clear
% dir = '/home/jiawei/Dropbox/mh_results/mh_01s';
dir = '~/.ros';
gt_a = load(strcat(dir,'/truth.txt'));
vo = load(strcat(dir,'/vo.txt'));
% vo(:,1) = vo(:,1) + 0.15;
% gt_a = gt_a(1:end-50, :);
% vo = vo(1:end-50, :);
% gt_a = gt_a(15:end, :);
% vo = vo(15:end, :);

% get overlap of gt with vo
i = 1; % index of vo
while(vo(i,1) < gt_a(1,1))
    i = i+1;
end
vo = vo(i:end, :);
i = 1;
j = 1; % index of gt_a
k = 1; % index of gt_a
while i<=size(vo,1)
    while(j<=size(gt_a,1) && vo(i,1) > gt_a(j,1))
        j = j+1;
    end
    if(j>size(gt_a,1))
        break;
    else
        gt(k,1) = vo(i,1);
        inc = (gt_a(j,2:4) - gt_a(j-1,2:4))/(gt_a(j,1) - gt_a(j-1,1));
        gt(k,2:4) = gt_a(j-1,2:4) + (vo(i,1) - gt_a(j-1,1))*inc;
    end
    i = i+1;
    k = k+1;
end
vo = vo(1:length(gt),:);

%% align two set of points
cg = mean(gt(:,2:4));
cv = mean(vo(:,3:5));
H = zeros(3,3);
for i=1:length(gt)
    H = H + (gt(i,2:4)-cg)'*(vo(i,3:5)-cv);
end
[U,S,V] = svd(H);
R = U*V';
for i=1:size(vo,1)
    p = R*vo(i,3:5)';
    vo(i,3:5) = p';
end
for i=size(vo,1):-1:1
    vo(i,3:5) = vo(i,3:5) - vo(1,3:5);
end
for i=size(gt,1):-1:1
    gt(i,2:4) = gt(i,2:4) - gt(1,2:4);
end

%% plot results
% position
gtp = gt(:,2:4);
vop = vo(:,3:5);
figure('Name','Trajectory')
plot3(gtp(:,1), gtp(:,2), gtp(:,3), 'g-')
hold on
plot3(vop(:,1), vop(:,2), vop(:,3), 'r-')
legend('Truth', 'Estimated');
title('Trajectory')

step = floor(length(vo) / (vo(end,1)-vo(1,1))); % step of 1 sec

% calculate vo translation
for i=1+step:size(vo,1)
    vo(i, 6:8) = (vo(i, 3:5) - vo(i-step, 3:5));
end
vo = vo(1+step:end, :);

% calculate gt translation
for i=1+step:size(gt,1)
    gt(i, 5:7) = (gt(i, 2:4) - gt(i-step, 2:4));
end
gt = gt(1+step:end, :);

% velocity scale
gtvn = vecnorm(gt(:,5:7)')';
vovn = vecnorm(vo(:,6:8)')';
di = vo(:,2)==0;
si = vo(:,2)==1;
diffn = abs(gtvn - vovn);
diffn_d = diffn(di);
diffn_s = diffn(si);
fprintf('RMSE offset of scale (DSVO, Stereo, Overall) = (%f, %f, %f)\n', ...
    sqrt(diffn_d' * diffn_d / length(diffn_d)), sqrt(diffn_s' * diffn_s / length(diffn_s)), sqrt(diffn' * diffn / length(diffn)));
fprintf('Median offset of scale (DSVO, Stereo, Overall) = (%f, %f, %f)\n', median(diffn_d), median(diffn_s), median(diffn));
fprintf('Average offset percentage of scale (DSVO, Stereo, Overall) = (%f, %f, %f)\n', ...
    mean(diffn_d ./ gtvn(di)), mean(diffn_s ./ gtvn(si)), mean(diffn ./ gtvn));

figure('Name','Velocity Scale')
subplot(3,1,1);
plot(gt(:,1), gtvn, 'g.');
hold on
plot(vo(di,1), vovn(di), 'r.');
hold on
plot(vo(si,1), vovn(si), 'b.');
legend('Truth', 'Estimated by DSVO', 'Estimated by Stereo Match');
xlabel('Time s'); ylabel('Velocity m/s');
subplot(3,1,2);
plot(vo(di,1), diffn(di), 'r.');
hold on 
plot(vo(si,1), diffn(si), 'b.');
legend('DSVO', 'Stereo Match');
xlabel('Time s'); ylabel('Velocity Error m/s');
subplot(3,1,3);
plot([0:length(sort(diffn_d))-1]/length(sort(diffn_d)), sort(diffn_d));
hold on
strmedian = ['\leftarrow Median = ',num2str(median(diffn_d))];
plot(0.5, median(diffn_d), 'r*');
text(0.52, median(diffn_d), strmedian);
title('Velocity Error Sorted, Estimated by DSVO');
xlabel('Percentage'); ylabel('Velocity Error m/s');

% % velocity orientation
% gtvd = gt(:,5:7) ./ gtvn;
% vovd = vo(:,6:8) ./ vovn;
% gt_vel_ori = [gt(:,1), gt(:,1), gt(:,1)];
% vo_vel_ori = [vo(:,1), vo(:,1), vo(:,1)];
% for i=2:size(vo,1)
%     gt_vel_ori(i,:) = acos( gtvd(i,:))/3.14159*180;
%     vo_vel_ori(i,:) = acos( vovd(i,:))/3.14159*180;
% end
% figure('Name','Velocity Direction');
% subplot(3,1,1);
% plot(gt(:,1), gt_vel_ori(:,1), 'g-');
% hold on
% plot(vo(:,1), vo_vel_ori(:,1), 'r-');
% title('x direction');
% legend('Truth', 'Estimated');
% subplot(3,1,2);
% plot(gt(:,1), gt_vel_ori(:,2), 'g-');
% hold on
% plot(vo(:,1), vo_vel_ori(:,2), 'r-');
% title('y direction');
% subplot(3,1,3);
% plot(gt(:,1), gt_vel_ori(:,3), 'g-');
% hold on
% plot(vo(:,1), vo_vel_ori(:,3), 'r-');
% title('z direction');


%% VO time

tt = load(strcat(dir,'/time.txt'));
t_f = tt(tt(:,1)==0,2:3);  % whole frame
t_stereo_rectify = tt(tt(:,1)==-1,2:3);  % stereo rectify
t_stereo = tt(tt(:,1)==3,2:3);  % stereo match
t_normal = tt(tt(:,1)==1,2:3);  % normal frame
t_dsvo = tt(tt(:,1)==2,2:3);  % keyframe
t_feature = tt(tt(:,1)==11,2:3);  % feature tracking
t_prap_dir = tt(tt(:,1)==121,2:3);  % pose propagation, direct
t_prap_rfof = tt(tt(:,1)==122,2:3);  % pose propagation, refine by OF
t_pts = tt(tt(:,1)==131,2:3);  % points reconstruction 
t_scale = tt(tt(:,1)==132,2:3);  % scale opt time

fprintf('\nAve. time for [%d] stereo match = %f\n', size(t_stereo,1), mean(t_stereo(:,2)));
fprintf('Ave. time for [%d] normal frame = %f\n', size(t_normal,1), mean(t_normal(:,2)));
fprintf('Ave. time for [%d] DSVO Keyframe = %f\n', size(t_dsvo,1), mean(t_dsvo(:,2)));
fprintf('Ave. time for [%d] frame = %f\n', size(t_f,1), mean(t_f(:,2)));
fprintf('\nAve. stereo rectify time = %f\n', mean(t_stereo_rectify(:,2)));
fprintf('Ave. feature tracking time = %f\n', mean(t_feature(:,2)));
fprintf('Ave. pose propagation(direct) time = %f\n', mean(t_prap_dir(:,2)));
fprintf('Ave. pose propagation(refine by OF) time = %f\n', mean(t_prap_rfof(:,2)));
fprintf('Ave. points reconstruction time = %f\n', mean(t_pts(:,2)));
fprintf('Ave. optimization time = %f\n', mean(t_scale(:,2)));
% figure('Name','VO running time')
% plot(t_normal(:,1), t_normal(:,2), 'g*-');
% hold on
% plot(t_dsvo(:,1), t_dsvo(:,2), 'b*-');
% hold on
% plot(t_scale(:,1), t_scale(:,2), 'r*-');
% ylabel('runing time / ms');
% legend('Normal frame', 'Keyframe', 'Scale optimization time');
