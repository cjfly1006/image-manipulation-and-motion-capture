clear all;
close all;
clc;

% Initialize webcam
cam = webcam;

% Define threshold values
R_min = 130;
R_max = 255;
G_min = 0;
G_max = 240;
B_min = 0;
B_max = 108;

% Create object to save the video
v = VideoWriter('TaskG', 'Uncompressed AVI');
v.FrameRate = 30;
open(v);

% Create window to display the results
hFig = figure;

% Start timer
tic;

while true
    % Capture frame from the webcam
    image = snapshot(cam);
    
    % Split image into RGB channels
    R = image(:,:,1);
    G = image(:,:,2);
    B = image(:,:,3);
    
    % Apply multiband thresholding
    R_bin = (R > R_min) & (R < R_max);
    G_bin = (G > G_min) & (G < G_max);
    B_bin = (B > B_min) & (B < B_max);
    
    % Combining binary masks
    threshold = R_bin & G_bin & B_bin;
    
    % Clean threshold
    borderSize = 1;
    paddedbinary = padarray(threshold, [borderSize borderSize], 1);
    se = strel('disk', 35, 4);
    paddedclean = imclose(paddedbinary, se);
    clean = paddedclean(borderSize+1:end-borderSize, borderSize+1:end-borderSize);
    
    % Sobel edge detection
    sobel_x = [-2 0 2; -2 0 2; -2 0 2];
    sobel_y = [-1 -2 -1; 0 0 0; 1 2 1];
    Gx = conv2(double(clean), sobel_x, 'same');
    Gy = conv2(double(clean), sobel_y, 'same');
    gradient_magnitude = sqrt(Gx.^2 + Gy.^2);
    edge_mask = gradient_magnitude;
    
    % Dilation for edge thickening
    se_dilate = strel('disk', 8);
    edge_mask = imdilate(edge_mask, se_dilate);
    
    % Superimpose edge mask on RGB image
    image_double = im2double(image);
    image_with_edge(:,:,1) = max(image_double(:,:,1), edge_mask);
    image_with_edge(:,:,2) = max(image_double(:,:,2), edge_mask);
    image_with_edge(:,:,3) = image_double(:,:,3) .* (1 - edge_mask);
    
    % Calculate the centroid and medoid
    labeled_clean = logical(clean);
    stats = regionprops(labeled_clean, 'Centroid', 'Area');
    [~, largestIdx] = max([stats.Area]);
    centroid = cat(1, stats(largestIdx).Centroid);
    
    % Resize clean image
    resizedImage = imresize(clean, [size(clean, 1) / 7, size(clean, 2) / 7]);
    [rows, cols] = find(resizedImage == 1);
    points = [rows, cols];
    numPoints = size(points, 1);
    distances = zeros(numPoints, numPoints);
    for i = 1:numPoints
        for j = 1:numPoints
            distances(i, j) = sqrt((points(i, 1) - points(j, 1))^2 + (points(i, 2) - points(j, 2))^2);
        end
    end
    sumDistances = sum(distances, 2);
    [~, medoidIndex] = min(sumDistances);
    medoid = points(medoidIndex, :);

    % Resize medoid coordinates to fit original image
    medoid(1) = medoid(1) * 7;
    medoid(2) = medoid(2) * 7;
    
    % Combine all results into a 2x2 montage
    % Original RGB image
    ax1 = subplot(2, 2, 1);
    imshow(image);
    title('Original Image');
    
    % Thresholded image
    ax2 = subplot(2, 2, 2);
    imshow(threshold);
    title('Thresholded Image');
    
    % Cleaned thresholded image with annotations
    ax3 = subplot(2, 2, 3);
    imshow(clean);
    hold on;
    plot(centroid(:, 1), centroid(:, 2), 'r.', 'MarkerSize', 20);
    plot(medoid(2), medoid(1), 'ro', 'MarkerSize', 10);
    title('Cleaned Image with Annotations');
    hold off;
    
    % Image with borders, centroid, and medoid applied to video stream
    ax4 = subplot(2, 2, 4);
    imshow(image_with_edge);
    hold on;
    plot(centroid(:, 1), centroid(:, 2), 'y.', 'MarkerSize', 20);
    plot(medoid(2), medoid(1), 'yo', 'MarkerSize', 10);
    title('Annotated Video Stream');
    hold off;
    
    % Capture the frame for the video
    frame = getframe(hFig);

    % Resize the frame
    frame_resized = imresize(frame.cdata, [647, 482]);
    
    % Write the frame to the video
    writeVideo(v, frame_resized);
    
    % Break the loop if the user closes the figure
    if ~ishandle(hFig)
        break;
    end

    % Break loop if 40 seconds have passed
    elapsedTime = toc;
    if elapsedTime > 40
        break;
    end
end

% Clean up and release the webcam
clear cam;
close(v);
