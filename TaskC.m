clear all;
close all;
clc;

% Reading image
image = imread('IM.bmp');
R = image(:,:,1);
G = image(:,:,2);
B = image(:,:,3);

% Setting threshold values
R_min = 130;
R_max = 255;
G_min = 0;
G_max = 240;
B_min = 0;
B_max = 108;

% Applying multiband thresholding
R_bin = (R > R_min) & (R < R_max);
G_bin = (G > G_min) & (G < G_max);
B_bin = (B > B_min) & (B < B_max);

% Combining binary masks
threshold = R_bin & G_bin & B_bin;

% Entry-wise product
image_double = im2double(image);
threshold3D = cat(3, threshold, threshold, threshold);
ew = image_double .* threshold3D;

% Results
figure;
subplot(1, 2, 1);
imshow(image);

subplot(1, 2, 2);
imshow(threshold);
imwrite(threshold, 'threshold.bmp');

figure;
subplot(1, 2, 2);
imshow(ew);
title('O');

