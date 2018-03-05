close all
clear
gt_a = load('~/.ros/truth.txt');
vo = load('~/.ros/vo.txt');
gt_a(:,2:4) = gt_a(:,2:4) - repmat(gt_a(2,2:4)-vo(2,2:4), size(gt_a,1), 1);

for i=2:size(vo,1)
    vo(i, 5:7) = (vo(i, 2:4) - vo(i-1, 2:4)) / (vo(i, 1) - vo(i-1, 1));
end
vo = vo(2:end, :);

for i=2:size(gt_a,1)
    gt_a(i, 5:7) = (gt_a(i, 2:4) - gt_a(i-1, 2:4)) / (gt_a(i, 1) - gt_a(i-1, 1));
end
gt = gt_a(2:end, :);

gtt = gt(:,1)-gt(1,1);
vot = vo(:,1)-vo(1,1);

gtvn = vecnorm(gt(:,5:7)')';
vovn = vecnorm(vo(:,5:7)')';
figure('Name','scale')
plot(gtt, gtvn, 'go-');
hold on
plot(vot, vovn, 'r*-');
% hold on
% plot(vot, log(vo(:,5)), 'b');
% hold on
% plot(vot, log(vo(:,6)-vo(:,7)), 'm');


gtvd = gt(:,5:7) ./ gtvn;
vovd = vo(:,5:7) ./ vovn;
gtc = gtt;
voc = vot;
for i=2:size(vo,1)
    gtc(i) = acos(gtvd(i-1) * gtvd(i))/3.14159*180;
    voc(i) = acos(vovd(i-1) * vovd(i))/3.14159*180;
end
figure('Name','direction')
plot(gtt, gtc, 'go-')
hold on
plot(vot, voc, 'r*-');

gtp = gt(:,2:4);
vop = vo(:,2:4);
figure('Name','traj')
plot3(gtp(:,1), gtp(:,2), gtp(:,3), 'g-')
% hold on
figure('Name','traj1')
plot3(vop(:,1), vop(:,2), vop(:,3), 'r-')