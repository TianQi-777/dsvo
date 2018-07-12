% load data
close all
clear
dir = '~/.ros';
gt = load(strcat(dir,'/truth.txt'));
dsvo = load(strcat(dir,'/vo.txt'));
dsvo = dsvo(:, [1,3,4,5]);
sptam = load(strcat(dir,'/sptam.txt'));
dsvo(:,1) = dsvo(:,1) + 0.15;
sptam(:,1) = sptam(:,1) + 0.15;
% gt = gt(500:end, :); dsvo = dsvo(500:end, :); sptam = sptam(500:end, :);
i = 1; % index of gt
while(gt(i,1) < dsvo(1,1) || gt(i,1) < sptam(1,1))
    i = i+1;
end
gt = gt(i:end, :);

disp('DSVO')
[gtp, dsvop, gtt, dsvot, gtvn, dsvovn] = alignGT(gt, dsvo);
dsvo_diffn = abs(gtvn - dsvovn);
dsvo_pern = dsvo_diffn ./ gtvn;
dsvo_pern = sum(dsvo_pern) / length(dsvo_pern);
fprintf('RMSE offset of scale = %f\n', sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1)));
fprintf('Median offset of scale = %f\n', median(dsvo_diffn));
fprintf('Average offset percentage of scale = %f\n', dsvo_pern);

disp('S-PTAM')
[~, sptamp, ~, sptamt, gtvn, sptamvn] = alignGT(gt, sptam);
sptam_diffn = abs(gtvn - sptamvn);
sptam_pern = sptam_diffn ./ gtvn;
sptam_pern = sum(sptam_pern) / length(sptam_pern);
fprintf('RMSE offset of scale = %f\n', sqrt(sptam_diffn' * sptam_diffn / size(sptam_diffn,1)));
fprintf('Median offset of scale = %f\n', median(sptam_diffn));
fprintf('Average offset percentage of scale = %f\n', sptam_pern);


figure('Name','Trajectory')
plot3(gtp(:,1), gtp(:,2), gtp(:,3), 'r-')
hold on
plot3(dsvop(:,1), dsvop(:,2), dsvop(:,3), 'g-')
hold on
plot3(sptamp(:,1), sptamp(:,2), sptamp(:,3), 'b-')
legend('Truth', 'DSVO', 'SPTAM');
title('Trajectory')

figure('Name','Velocity Scale')
subplot(3,1,1);
plot(gtt, gtvn, 'r-');
hold on
plot(dsvot, dsvovn, 'g-');
hold on
plot(sptamt, sptamvn, 'b-');
ylim([0 min(5,max(max(dsvovn), max(sptamvn))+0.1)]);
legend('Truth', 'DSVO', 'SPTAM');
title('Scale per second');
subplot(3,1,2);
plot(dsvot, dsvo_diffn, 'g');
hold on
plot(sptamt, sptam_diffn, 'b');
legend('DSVO', 'SPTAM');
title('Absolute scale error');
subplot(3,1,3);
dsvo_sortn = sort(dsvo_diffn);
sptam_sortn = sort(sptam_diffn);
plot(dsvo_sortn, 'g');
hold on
plot(sptam_sortn, 'b');
legend('DSVO', 'SPTAM');
title('Sorted absolute scale error');
perct = dsvo_sortn(floor(0.95*length(dsvo_sortn)));
ylim([0 perct]);

function [gtp, vop, gtt, vot, gtvn, vovn] = alignGT(gt, vo)
% get overlap of gt with vo
i = 1; % index of gt
j = 1; % index of vo
k = 1; % index of vo
while i<=size(gt,1)
    while(j<=size(vo,1) && gt(i,1) > vo(j,1))
        j = j+1;
    end
    if(j>size(vo,1))
        break;
    else
        vo_n(k,1) = gt(i,1);
        if(gt(i,1) ~= vo(j,1))
            inc = (vo(j,2:4) - vo(j-1,2:4))/(vo(j,1) - vo(j-1,1));
            vo_n(k,2:4) = vo(j-1,2:4) + (gt(i,1) - vo(j-1,1))*inc;
        else
            vo_n(k,2:4) = vo(j,2:4);
        end
    end
    i = i+1;
    k = k+1;
end
% use range of idx
idx = 1:size(vo_n,1);
% idx = [size(gt,1):-1:size(gt,1)-200];
vo = vo_n(idx, :);
gt = gt(idx, :);

%% align two set of points
cg = mean(gt(:,2:4));
cv = mean(vo(:,2:4));
H = zeros(3,3);
for i=1:length(gt)
    H = H + (gt(i,2:4)-cg)'*(vo(i,2:4)-cv);
end
[U,~,V] = svd(H);
R = U*V';
for i=1:size(vo,1)
    p = R*vo(i,2:4)';
    vo(i,2:4) = p';
end
cg = mean(gt(:,2:4));
cv = mean(vo(:,2:4));
for i=size(vo,1):-1:1
    vo(i,2:4) = vo(i,2:4) - cv;
end
for i=size(gt,1):-1:1
    gt(i,2:4) = gt(i,2:4) - cg;
end

%% plot results
step = floor(length(vo) / (vo(end,1)-vo(1,1))); % step of 1 sec

% calculate vo translation
for i=1+step:size(vo,1)
    vo(i, 5:7) = (vo(i, 2:4) - vo(i-step, 2:4));
end
vo = vo(1+step:end, :);

% calculate gt translation
for i=1+step:size(gt,1)
    gt(i, 5:7) = (gt(i, 2:4) - gt(i-step, 2:4));
end
gt = gt(1+step:end, :);
 
% time axis
gtt = gt(:,1)-gt(1,1);
vot = vo(:,1)-vo(1,1);

% position
gtp = gt(:,2:4);
vop = vo(:,2:4);

% velocity scale
gtvn = vecnorm(gt(:,5:7)')';
vovn = vecnorm(vo(:,5:7)')';

% velocity orientation
gtvd = gt(:,5:7) ./ gtvn;
vovd = vo(:,5:7) ./ vovn;
gt_vel_ori = [gtt, gtt, gtt];
vo_vel_ori = [vot, vot, vot];
for i=2:size(vo,1)
    gt_vel_ori(i,:) = acos( gtvd(i,:))/3.14159*180;
    vo_vel_ori(i,:) = acos( vovd(i,:))/3.14159*180;
end
% figure('Name','Velocity Direction');
% subplot(3,1,1);
% plot(gtt, gt_vel_ori(:,1), 'g-');
% hold on
% plot(vot, vo_vel_ori(:,1), 'r-');
% title('x direction');
% legend('Truth', 'Estimated');
% subplot(3,1,2);
% plot(gtt, gt_vel_ori(:,2), 'g-');
% hold on
% plot(vot, vo_vel_ori(:,2), 'r-');
% title('y direction');
% subplot(3,1,3);
% plot(gtt, gt_vel_ori(:,3), 'g-');
% hold on
% plot(vot, vo_vel_ori(:,3), 'r-');
% title('z direction');

end
