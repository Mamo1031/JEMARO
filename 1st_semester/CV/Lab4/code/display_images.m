function display_images(image_files)
    % Display images in grayscale and split them into RGB and HSV channels
    for i = 1:length(image_files)
        img = imread(image_files{i});
        
        % Convert to grayscale
        gray_img = rgb2gray(img);

        
        % Split into RGB channels
        R = img(:, :, 1);
        G = img(:, :, 2);
        B = img(:, :, 3);
        
        % Convert to HSV and split
        hsv_img = rgb2hsv(img);
        H = hsv_img(:, :, 1);
        S = hsv_img(:, :, 2);
        V = hsv_img(:, :, 3);
        
        % Display the images
        figure;
        subplot(1, 2, 1); imshow(img); title('Original');
        subplot(1, 2, 2); imshow(gray_img); title('Grayscale');
        
        figure;
        subplot(2, 3, 1); imshow(R); title('Red Channel');
        subplot(2, 3, 2); imshow(G); title('Green Channel');
        subplot(2, 3, 3); imshow(B); title('Blue Channel');
        subplot(2, 3, 4); imshow(H); title('Hue Channel');
        subplot(2, 3, 5); imshow(S); title('Saturation Channel');
        subplot(2, 3, 6); imshow(V); title('Value Channel');
    end
end
