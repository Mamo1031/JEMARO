addpath('functions');

% Step 1: Implement Laplacian of Gaussian (LoG) operator
log_filter(1); % sd = 1
log_filter(3.5); % sd = 3.5

img = imread('../Lab3_testimages/boccadasse.jpg');
img = im2double(rgb2gray(img));


% Convolve the image with the LoG filters
filtered_img1 = imfilter(img, fspecial('log', [9 9], 1), 'replicate'); % sd = 1
filtered_img2 = imfilter(img, fspecial('log', [13 13], 3.5), 'replicate'); % sd = 3.5

figure;
subplot(1,2,1), imshow(filtered_img1, []), title('LoG Filtered Image (sd=1)');
subplot(1,2,2), imshow(filtered_img2, []), title('LoG Filtered Image (sd=3.5)');


% Apply edge detection and display results
edges_img1 = edge_detection_log(img, 1, 0);
edges_img2 = edge_detection_log(img, 1, 0.1);
edges_img3 = edge_detection_log(img, 1.5, 0.1);

figure;
subplot(1,3,1), imshow(edges_img1, []), title('Edges (sd=1, threshold=0)');
subplot(1,3,2), imshow(edges_img2, []), title('Edges (sd=1, threshold=0.1)');
subplot(1,3,3), imshow(edges_img3, []), title('Edges (sd=1.5, threshold=0.1)');


% Compare with MATLAB's edge function
edges_matlab = edge(img, 'log');
figure;
imshow(edges_matlab);
title('Edges using MATLAB edge function (log)');
pause(1);



% Step 2: Detect straight lines in images using Hough Transform
detect_lines('../Lab3_testimages/highway1.jpg', 4, [21 21]);
pause(1);
detect_lines('../Lab3_testimages/highway2.jpg', 5, [21 21]);
