function check_variation(image_files)
    % Dark car area
    region_x = 390:400;
    region_y = 575:595;
    
    % Check the compositional variation for each image
    rgb_means = [];
    hue_means = [];
    
    for i = 1:length(image_files)
        img = imread(image_files{i});
        
        % Get the RGB components of the specified area
        R = img(region_x, region_y, 1);
        G = img(region_x, region_y, 2);
        B = img(region_x, region_y, 3);
        
        % Calculate the average value of RGB components
        R_mean = mean(R(:));
        G_mean = mean(G(:));
        B_mean = mean(B(:));
        rgb_means = [rgb_means; R_mean, G_mean, B_mean];
        
        % Convert to HSV to get the hue component
        hsv_img = rgb2hsv(img);
        hue = hsv_img(region_x, region_y, 1);
        
        % Calculate the average value of the Hue components
        hue_mean = mean(hue(:));
        hue_means = [hue_means; hue_mean];
        
        % Displays the average value of RGB and Hue components of each image
        fprintf('Image %d - R: %.2f, G: %.2f, B: %.2f, Hue: %.2f\n', ...
            i, R_mean, G_mean, B_mean, hue_mean);
    end
    
    % Plotting the variation of RGB components
    figure;
    subplot(2, 1, 1);
    plot(1:length(image_files), rgb_means(:, 1), '-r', 'DisplayName', 'Red');
    hold on;
    plot(1:length(image_files), rgb_means(:, 2), '-g', 'DisplayName', 'Green');
    plot(1:length(image_files), rgb_means(:, 3), '-b', 'DisplayName', 'Blue');
    title('RGB Components Variation');
    xlabel('Image Number');
    ylabel('Mean Value');
    legend;
    
    % Plot the variation of Hue components
    subplot(2, 1, 2);
    plot(1:length(image_files), hue_means, '-m', 'DisplayName', 'Hue');
    title('Hue Component Variation');
    xlabel('Image Number');
    ylabel('Mean Value');
    legend;
end
