clear all;
close all;
clc;

% Reading image
image = imread('IM.bmp');
binary = imread('threshold.bmp');

% Creating padding to avoid isues with text along the border
borderSize = 1;
paddedbinary = padarray(binary, ...
    [borderSize borderSize], 1);

% Create a strel for morphological closing
se = strel('disk', 45, 4);
paddedclean = imclose(paddedbinary, se);

% Removing padding
clean = paddedclean(borderSize+1:end-borderSize, borderSize+1:end-borderSize);

% Entry-wise product
image_double = im2double(image);
clean3D = cat(3, clean, clean, clean);
ew = image_double .* clean3D;

% Results
figure;
subplot(1, 2, 1);
imshow(clean)
imwrite(clean, 'clean_bw.bmp')

subplot(1, 2, 2);
imshow(ew);
imwrite(ew, 'clean_rgb.bmp')
