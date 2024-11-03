function segment_dark_car(image_files)
    % Select the dark car area in the first image and compute mean and std of Hue
    img = imread('../Lab4_testimages/ur_c_s_03a_01_L_0376.png');
    hsv_img = rgb2hsv(img);
    hue = hsv_img(:, :, 1);
    
    % Select the region corresponding to the dark car
    dark_car_area = hue(390:400, 575:595);
    m = mean(dark_car_area(:));
    s = std(dark_car_area(:));
    
    % Perform segmentation for all 6 images
    for i = 1:length(image_files)
        img = imread(image_files{i});
        hsv_img = rgb2hsv(img);
        hue = hsv_img(:, :, 1);
        
        % Thresholding based on mean and standard deviation
        mask = (hue >= (m - s)) & (hue <= (m + s));
        
        % Find the largest blob
        props = regionprops(mask, 'Area', 'Centroid', 'BoundingBox');
        [~, max_idx] = max([props.Area]);
        largest_blob = props(max_idx);
        
        % Display binary image and bounding box
        figure;
        subplot(1, 2, 1);
        imshow(mask);
        hold on;
        rectangle('Position', largest_blob.BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
        plot(largest_blob.Centroid(1), largest_blob.Centroid(2), 'r*');
        title('Dark Car Segmentation');
        
        % Display bounding box on the original image
        subplot(1, 2, 2);
        imshow(img);
        hold on;
        rectangle('Position', largest_blob.BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
        plot(largest_blob.Centroid(1), largest_blob.Centroid(2), 'r*');
        title('Dark Car on Original Image');
    end
end
