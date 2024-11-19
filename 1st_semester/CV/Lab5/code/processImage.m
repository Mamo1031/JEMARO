function processImage(imagePath, template, carName, imageIndex)
% Function to process an image and draw the result

    img = imread(imagePath);
    gray_img = im2gray(img);

    % NCC
    score_map = normxcorr2(template, gray_img);

    % Score Maximum
    [max_val, max_index] = max(abs(score_map(:)));
    [y_peak, x_peak] = ind2sub(size(score_map), max_index);

    % Ajust coordenates to og system 
    x_offset = x_peak - size(template, 2);
    y_offset = y_peak - size(template, 1);

    % limits
    x_offset = max(1, x_offset);
    y_offset = max(1, y_offset);

    x_star = x_offset + size(template, 2) / 2;
    y_star = y_offset + size(template, 1) / 2;

    % Exibit image
    pause(1.0);
    figure, imshow(gray_img);
    hold on;
    rectangle('Position', [x_offset, y_offset, size(template, 2), size(template, 1)], ...
              'EdgeColor', 'r');
    plot(x_star, y_star, 'r*');
    title([carName, ' - Image ', num2str(imageIndex)]);
    hold off;

end
