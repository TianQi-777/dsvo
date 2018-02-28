close all
clear
gt_a = load('~/.ros/truth.txt');
vo = load('~/.ros/vo.txt');
gt_a(:,2:4) = gt_a(:,2:4) - repmat(gt_a(2,2:4)-vo(2,2:4), length(gt_a), 1);

vo_cols = size(vo,2);
for i=2:length(vo)
    vo(i, vo_cols+1:vo_cols+3) = (vo(i, 2:4) - vo(i-1, 2:4)) / (vo(i, 1) - vo(i-1, 1));
end
vo = vo(2:end, :);

for i=2:length(gt_a)
    gt_a(i, 5:7) = (gt_a(i, 2:4) - gt_a(i-1, 2:4)) / (gt_a(i, 1) - gt_a(i-1, 1));
end
gt = gt_a(2:end, :);

gtt = gt(:,1)-gt(1,1);
vot = vo(:,1)-vo(1,1);

gtvn = vecnorm(gt(:,5:7)')';
vovn = vecnorm(vo(:,vo_cols+1:vo_cols+3)')';
plot(gtt, gtvn, 'go-');
hold on
plot(vot, vovn, 'r*-');
% hold on
% plot(vot, log(vo(:,5)), 'b');
% hold on
% plot(vot, log(vo(:,6)-vo(:,7)), 'm');


% gtvd = gt(:,5:7) ./ gtvn;
% vovd = vo(:,vo_cols+1:vo_cols+3) ./ vovn;
% gtc = gtt;
% voc = vot;
% for i=2:length(vo)
%     gtc(i) = gtvd(i-1) * gtvd(i);
%     voc(i) = vovd(i-1) * vovd(i);
% end
% figure 
% plot(gtt, gtc, 'g*-')
% hold on
% plot(vot, voc, 'r*-');
% 
% gtp = gt(:,2:4);
% vop = vo(:,2:4);
% figure 
% plot3(gtp(:,2), -gtp(:,3), -gtp(:,1))
% hold on
% plot3(vop(:,1), vop(:,2), vop(:,3))