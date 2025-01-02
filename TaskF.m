clear all;
close all;
clc;

% Reading images
image = imread('IM.bmp');
clean = imread('clean_bw.bmp');

labeled_clean = logical(clean);

% Finding and extracting the largest centroid
stats = regionprops(labeled_clean,'Centroid', 'Area');
[~, largestIdx] = max([stats.Area]);
centroid = cat(1, stats(largestIdx).Centroid);

% Extracting coordinates of foreground points
resizedImage = imresize(clean, [(3088/200), (2316/200)]);   % Resizing image by dividing by 200 as clean image is to large
[rows, cols] = find(resizedImage == 1);
points = [rows, cols];

% Computing and storing pairwise distances between all points
numPoints = size(points, 1);
distances = zeros(numPoints, numPoints);

% Calculateing distance
for i = 1:numPoints
    for j = 1:numPoints
        distances(i, j) = sqrt((points(i, 1) - points(j, 1))^2 ...
            + (points(i, 2) - points(j, 2))^2);
    end
end

% Calculating the sum of distances
sumDistances = sum(distances, 2);

% Indexing medioid
[~, medoidIndex] = min(sumDistances);

% Resizing medoid to fit on clean image
medoid = points(medoidIndex, :);
medoid(1) = medoid(1).*200;
medoid(2) = medoid(2).*200;


% Ploting the centroid and medoid
figure;
imshow(clean);
hold on;

plot(centroid(:, 1), centroid(:, 2), 'r.', 'MarkerSize', 20);
plot(medoid(2), medoid(1), 'ro', 'MarkerSize', 10);

figure;
imshow(image);
hold on;

plot(centroid(:, 1), centroid(:, 2), 'y.', 'MarkerSize', 20);
plot(medoid(2), medoid(1), 'yo', 'MarkerSize', 10);
