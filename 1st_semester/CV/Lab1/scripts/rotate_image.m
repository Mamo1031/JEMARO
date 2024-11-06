function output_img = rotate_image(input_img, angle)
    % Get the size of the input image
    [rows, cols, ch] = size(input_img);
    
    % Create a grid of coordinates for the output image
    [X, Y] = meshgrid(1:cols, 1:rows);
    
    % Calculate the center of the image
    center_x = cols / 2;
    center_y = rows / 2;
    
    % Convert angle to radians
    angle_rad = deg2rad(angle);
    
    % Calculate the rotation matrix
    R = [cos(angle_rad) -sin(angle_rad); sin(angle_rad) cos(angle_rad)];
    
    % Shift the coordinates to the center, apply the rotation, and shift back
    X_shifted = X - center_x;
    Y_shifted = Y - center_y;
    rotated_coords = R * [X_shifted(:)'; Y_shifted(:)'];
    X_rotated = reshape(rotated_coords(1, :) + center_x, size(X));
    Y_rotated = reshape(rotated_coords(2, :) + center_y, size(Y));
    
    % Initialize the output image with zeros
    output_img = zeros(rows, cols, ch, 'like', input_img);
    
    % Perform backward mapping and bilinear interpolation
    for c = 1:ch
        output_img(:, :, c) = interp2(double(input_img(:, :, c)), X_rotated, Y_rotated, 'linear', 0);
    end
    
    rotated_img = output_img;
    
    figure;
    imshow(rotated_img);
    title(['Rotated Image by ' num2str(angle) ' Degrees']);
    pause(0.5);
end
