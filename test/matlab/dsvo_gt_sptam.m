% load data
close all
clear
dir = '~/.ros';
dsvo_t = load(strcat(dir,'/time.txt'));
dsvo_t = dsvo_t(dsvo_t(:,1)==0,3);  % whole frame
sptam_t = load(strcat(dir,'/sptam_time.txt'));
gt = load(strcat(dir,'/truth.txt'));
dsvo = load(strcat(dir,'/vo.txt'));
dsvo = dsvo(:, [1,3,4,5]);
sptam = load(strcat(dir,'/sptam.txt'));
dsvo(:,1) = dsvo(:,1) + 0.15;
sptam(:,1) = sptam(:,1) + 0.15;
gt = gt(500:end, :); dsvo = dsvo(500:end, :); sptam = sptam(500:end, :);
i = 1; % index of gt
while(gt(i,1) < dsvo(1,1) || gt(i,1) < sptam(1,1))
    i = i+1;
end
gt = gt(i:end, :);

[gtp, dsvop, gtt, dsvot, gtvn, dsvovn, dsvo_diffo] = alignGT(gt, dsvo);
[~, sptamp, ~, sptamt, ~, sptamvn, sptam_diffo] = alignGT(gt, sptam);
dsvo_diffn = abs(gtvn - dsvovn);
sptam_diffn = abs(gtvn - sptamvn);
Method = {'DSVO'; 'S-PTAM'};
scale_RMSE = [sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1));sqrt(sptam_diffn' * sptam_diffn / size(sptam_diffn,1))]; 
scale_Median = [median(dsvo_diffn); median(sptam_diffn)];
direction_RMSE = [sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1)); sqrt(sptam_diffo' * sptam_diffo / size(sptam_diffo,1))];
direction_Median = [median(dsvo_diffn); median(sptam_diffo)];
time_Mean = [mean(dsvo_t); mean(sptam_t)];

Result = table(Method, scale_RMSE, scale_Median, direction_RMSE, direction_Median, time_Mean);
disp(Result)

figure('Name','Trajectory')
plot3(dsvop(:,1), dsvop(:,2), dsvop(:,3), 'g-')
hold on
plot3(sptamp(:,1), sptamp(:,2), sptamp(:,3), 'b-')
hold on
plot3(gtp(:,1), gtp(:,2), gtp(:,3), 'r-')
legend('DSVO', 'SPTAM', 'Truth');
axis equal
title('Trajectory')
% 
% disp('DSVO')
% fprintf('RMSE offset of scale = %f\n', sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1)));
% fprintf('Median offset of scale = %f\n', median(dsvo_diffn));
% fprintf('RMSE offset of direction = %f\n', sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1)));
% fprintf('Median offset of direction = %f\n', median(dsvo_diffn));
% 
% disp('S-PTAM')
% fprintf('RMSE offset of scale = %f\n', sqrt(sptam_diffn' * sptam_diffn / size(sptam_diffn,1)));
% fprintf('Median offset of scale = %f\n', median(sptam_diffn));
% fprintf('RMSE offset of direction = %f\n', sqrt(sptam_diffo' * sptam_diffo / size(sptam_diffo,1)));
% fprintf('Median offset of direction = %f\n', median(sptam_diffo));


% 
figure('Name','Velocity')
% subplot(1,2,1);
plot(dsvot, dsvovn, 'r-');
hold on
plot(sptamt, sptamvn, 'b-');
hold on
plot(gtt, gtvn, 'g-');
ylim([0 min(5,max(max(dsvovn), max(sptamvn))+0.1)]);
legend('DSVO', 'SPTAM', 'Truth');
xlabel('Time s'); ylabel('Velocity m/s');
title('Velocity Scale');
% plot(dsvot, dsvo_diffn, 'g-');
% hold on
% plot(sptamt, sptam_diffn, 'b-');
% legend('DSVO', 'SPTAM');
% xlabel('Time s'); ylabel('Velocity Error m/s');

% subplot(1,2,2);
% plot(dsvot, dsvo_diffo, 'g-');
% hold on
% plot(sptamt, sptam_diffo, 'b-');
% legend('DSVO', 'SPTAM');
% xlabel('Time s'); ylabel('Degree');
% title('Velocity Direction');
