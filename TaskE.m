clear all;
close all;
clc;

% Reading images
image = imread('IM.bmp');
clean = imread('clean_bw.bmp');

% Defining Sobel filters for horizontal and vertical edge detection
sobel_x = [-2 0 2; -2 0 2; -2 0 2];
sobel_y = [-1 -2 -1; 0 0 0; 1 2 1];

% Convolving the binary mask with Sobel filters
Gx = conv2(double(clean), sobel_x, 'same');
Gy = conv2(double(clean), sobel_y, 'same');
gradient_magnitude = sqrt(Gx.^2 + Gy.^2);

% Threshold the gradient magnitude to create edge map
edge_mask = gradient_magnitude;

% Makeing the outline thicker using morphological dilation
se_dilate = strel('disk', 8);
edge_mask = imdilate(edge_mask, se_dilate);

% Super impose yellow boundary to RGB image
image_double = im2double(image);
image_with_edge(:,:,1) = max(image_double(:,:,1), edge_mask);
image_with_edge(:,:,2) = max(image_double(:,:,2), edge_mask);
image_with_edge(:,:,3) = image_double(:,:,3) .* (1 - edge_mask);

% Results
figure;
subplot(1, 3, 1);
imshow(edge_mask);
imwrite(edge_mask, 'boundry_image.bmp')

subplot(1, 3, 2);
imshow(image_with_edge);
imwrite(image_with_edge, 'boundry_rgb_image.bmp')
