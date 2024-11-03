function segment_red_car(image_files)
    % Segment the red car using Hue values
    for i = 1:length(image_files)
        img = imread(image_files{i});
        hsv_img = rgb2hsv(img);
        hue = hsv_img(:, :, 1);
        
        % Thresholding the hue for red color (e.g. hue > 0.97 and < 1)
        mask = (hue > 0.97) & (hue < 1);
        
        % Find the largest blob
        props = regionprops(mask, 'Area', 'Centroid', 'BoundingBox');
        [~, max_idx] = max([props.Area]);
        largest_blob = props(max_idx);
        
        % Display binary image and bounding box
        figure;
        subplot(1, 2, 1);
        imshow(mask);
        hold on;
        rectangle('Position', largest_blob.BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
        plot(largest_blob.Centroid(1), largest_blob.Centroid(2), 'g*');
        title('Red Car Segmentation');
        
        % Display bounding box on the original image
        subplot(1, 2, 2);
        imshow(img);
        hold on;
        rectangle('Position', largest_blob.BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
        plot(largest_blob.Centroid(1), largest_blob.Centroid(2), 'g*');
        title('Red Car on Original Image');
    end
end
