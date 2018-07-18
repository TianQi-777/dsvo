close all
clear
dir = '~/.ros';
tt = load(strcat(dir,'/sptam_time.txt'));

mean(tt)