% Lab7
clc;
close all;
clear;

%% Part 1
compareCDAlgo('../video/luce_vp.mp4', 30, 0.1, 15);

compareCDOF('../video/tennis.mp4', 0.5, 0.1, 0.3, 20);

%% Part 2
segmentAndTrack('../video/DibrisHall.mp4', 0.5, 0.05, 0.1) 
