function blob_detection()
    sunflower_image = imread('../Lab4_testimages/sunflowers.png');
    sunflower_image = double(sunflower_image);

    % Coordinates of two highlighted sunflowers (from Fig.2)
    sunflower1_y = 335:385;
    sunflower1_x = 140:190;
    sunflower2_y = 340:420;
    sunflower2_x = 425:495;
    
    % Extract the regions of interest (ROIs) for the two sunflowers
    sunflower1 = sunflower_image(sunflower1_y, sunflower1_x);
    sunflower2 = sunflower_image(sunflower2_y, sunflower2_x);
    
    % Display the original image with bounding boxes for the selected areas
    figure;
    imshow(uint8(sunflower_image));
    hold on;
    
    % Calculate the width and height using max and min instead of range
    width1 = max(sunflower1_x) - min(sunflower1_x);
    height1 = max(sunflower1_y) - min(sunflower1_y);
    width2 = max(sunflower2_x) - min(sunflower2_x);
    height2 = max(sunflower2_y) - min(sunflower2_y);
    
    % Draw rectangles around the selected sunflower regions
    rectangle('Position', [min(sunflower1_x), min(sunflower1_y), width1, height1], ...
              'EdgeColor', 'r', 'LineWidth', 2);
    rectangle('Position', [min(sunflower2_x), min(sunflower2_y), width2, height2], ...
              'EdgeColor', 'r', 'LineWidth', 2);
    
    title('Selected Sunflower Regions');
    hold off;
    
    % Compute Laplacian responses for each scale
    start_sigma = 1;
    num_scales = 10;
    sigma_increment = 1.5;
    
    % Store the maximum Laplacian responses for each sunflower
    max_response1 = zeros(1, num_scales);
    max_response2 = zeros(1, num_scales);
    
    figure;
    hold on;
    
    % Loop over scales to compute the Laplacian response for each sunflower
    for i = 1:num_scales
        sigma = start_sigma * (sigma_increment ^ (i - 1));
        
        % Apply Gaussian filter with varying sigma
        laplacian1 = imgaussfilt(sunflower1, sigma, 'FilterSize', 3);
        laplacian2 = imgaussfilt(sunflower2, sigma, 'FilterSize', 3);
        
        % Compute Laplacian of Gaussian-filtered image
        laplacian_response1 = del2(laplacian1);
        laplacian_response2 = del2(laplacian2);
        
        % Find maximum Laplacian response for each sunflower
        max_response1(i) = max(laplacian_response1(:));
        max_response2(i) = max(laplacian_response2(:));
    end
    
    % Plot the Laplacian response as a function of scale for both sunflowers
    plot(1:num_scales, max_response1, '-bo', 'DisplayName', 'Sunflower 1');
    plot(1:num_scales, max_response2, '-ro', 'DisplayName', 'Sunflower 2');
    title('Laplacian Response as a Function of Scale');
    xlabel('Scale');
    ylabel('Max Laplacian Response');
    legend;
    
    % Find the characteristic scale for each sunflower
    [~, char_scale1] = max(max_response1);
    [~, char_scale2] = max(max_response2);
    
    % Display characteristic scale (in terms of scale index and corresponding sigma)
    fprintf('Sunflower 1 - Characteristic Scale: %d (Sigma = %.2f)\n', char_scale1, start_sigma * (sigma_increment ^ (char_scale1 - 1)));
    fprintf('Sunflower 2 - Characteristic Scale: %d (Sigma = %.2f)\n', char_scale2, start_sigma * (sigma_increment ^ (char_scale2 - 1)));
    
    hold off;
end
