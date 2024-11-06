function output_img = translate_image(input_img, tx, ty)
    % Get the size of the input image
    [rows, cols, ch] = size(input_img);
    
    % Create a grid of coordinates for the output image
    [X, Y] = meshgrid(1:cols, 1:rows);
    
    % Apply translation
    X_translated = X - tx;
    Y_translated = Y - ty;
    
    % Initialize the output image with zeros
    output_img = zeros(rows, cols, ch, 'like', input_img);
    
    % Perform backward mapping and bilinear interpolation
    for c = 1:ch
        output_img(:, :, c) = interp2(double(input_img(:, :, c)), X_translated, Y_translated, 'linear', 0);
    end
    
    translated_img = output_img;
    
    
    figure;
    imshow(translated_img);
    title('Translated Image');
    pause(0.5);
end
