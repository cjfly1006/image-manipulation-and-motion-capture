clear all;
close all;
clc;

% Initialize webcam
cam = webcam;

% Threshold values
R_min = 130; R_max = 255;
G_min = 0;   G_max = 240;
B_min = 0;   B_max = 108;

% Initialize video writer
v = VideoWriter('TaskH', 'Uncompressed AVI');
v.FrameRate = 10;
open(v);

% Initialize figure for display
hFig = figure;

% Previous centroid coordinates gesture detection
prevCentroid = [0, 0];
prevBorder = [0, 0, 0, 0];
gesture = '';

% Start timer
tic;

while true
    % Capture frame
    image = snapshot(cam);
    
    % Split into RGB channels
    R = image(:, :, 1); G = image(:, :, 2); B = image(:, :, 3);
    
    % Thresholding
    R_bin = (R > R_min) & (R < R_max);
    G_bin = (G > G_min) & (G < G_max);
    B_bin = (B > B_min) & (B < B_max);
    threshold = R_bin & G_bin & B_bin;
    
    % Clean thresholded image
    se = strel('disk', 25);
    clean = imclose(threshold, se);
    
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

    % Create yellow overlay for the edge mask
    overlay = cat(3, edge_mask > 0, edge_mask > 0, zeros(size(edge_mask)));
    clean_rgb = repmat(clean, [1, 1, 3]);
    
    % Blend clean image with edge mask overlay
    blended_image = im2double(clean_rgb);
    blended_image(overlay > 0) = max(blended_image(overlay > 0), 1);

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

    % Find centroid and border
    stats = regionprops(clean, 'Centroid', 'BoundingBox', 'Area');
    if ~isempty(stats)
        [~, largestIdx] = max([stats.Area]);
        centroid = stats(largestIdx).Centroid;
        bbox = stats(largestIdx).BoundingBox;
        border = [bbox(2), bbox(2) + bbox(4), bbox(1), bbox(1) + bbox(3)];
        area = stats(largestIdx).Area;
    else
        centroid = [0, 0];
        border = [0, 0, 0, 0];
        area = 0;
    end
    
    % Detect gesture based on centroid, border movement, and area change
    if all(prevCentroid ~= 0) && all(prevBorder ~= 0)
        diffCentroid = centroid - prevCentroid;
        areaChange = area - prevArea;
        
        % Classify gestures
        if abs(diffCentroid(1)) > 30 || abs(diffCentroid(2)) > 30
            angle = atan2d(diffCentroid(2), diffCentroid(1));

            if angle > -22.5 && angle <= 22.5
                gesture = 'Right Swipe';
            elseif angle > 22.5 && angle <= 67.5
                gesture = 'Down-Right Swipe';
            elseif angle > 67.5 && angle <= 112.5
                gesture = 'Down Swipe';
            elseif angle > 112.5 && angle <= 157.5
                gesture = 'Down-Left Swipe';
            elseif (angle > 157.5 && angle <= 180) || (angle >= -180 && angle <= -157.5)
                gesture = 'Left Swipe';
            elseif angle > -157.5 && angle <= -112.5
                gesture = 'Up-Left Swipe';
            elseif angle > -112.5 && angle <= -67.5
                gesture = 'Up Swipe';
            elseif angle > -67.5 && angle <= -22.5
                gesture = 'Up-Right Swipe';
            end
        elseif areaChange > 5000
            gesture = 'Foam Finger Closer';
        elseif areaChange < -5000
            gesture = 'Foam Finger Further';
        end
    end
    
    % Update previous centroid, border, and area
    prevCentroid = centroid;
    prevBorder = border;
    prevArea = area;

    % Annotate the images
    annotatedImage = insertText(image, [10, 10], gesture, 'FontSize', 18, 'BoxColor', 'yellow', 'TextColor', 'black');
    edge_mask_scaled = edge_mask / max(edge_mask(:));
    overlayEdge = imoverlay(annotatedImage, edge_mask_scaled > 0.5, [1, 1, 0]);

    % Display and annotate montage
    subplot(2, 2, 1)
    imshow(image)
    title('Original Image');
    
    subplot(2, 2, 2)
    imshow(threshold)
    title('Thresholded Image');
    
    subplot(2, 2, 3)
    imshow(blended_image)
    title('Clean with Edge Mask');
    hold on;
    plot(centroid(1), centroid(2), 'r*', 'MarkerSize', 10);
    plot(medoid(2), medoid(1), 'g*', 'MarkerSize', 10);
    hold off;
    
    subplot(2, 2, 4)
    imshow(overlayEdge)
    title(['Gesture: ', gesture]);
    hold on;
    plot(centroid(1), centroid(2), 'r*', 'MarkerSize', 10);
    hold off;

    % Capture the frame
    frame = getframe(hFig);

    % Resize the frame
    frame_resized = imresize(frame.cdata, [647, 482]);
    
    % Write the frame to the video
    writeVideo(v, frame_resized);

    % Break if figure is closed
    if ~ishandle(hFig)
        break; 
    end

    % Stop after 40 seconds
    elapsedTime = toc;
    if elapsedTime > 40
        break;
    end
end

% Clean up
clear cam;
close(v);
