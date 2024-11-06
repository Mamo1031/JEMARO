function output_img = warp_image(input_img, XD, YD)
    % Get the size of XD and YD
    [rows, cols, ch] = size(input_img);

    [X, Y] = meshgrid(1:cols, 1:rows);
    
    if size(XD, 1) ~= rows || size(XD, 2) ~= cols
        error('XD and YD must be the same size as the input image.');
    end
    
    output_img = zeros(rows, cols, ch, 'like', input_img);
    
    % Adjust XD and YD to be within the valid range of the input image
    XD = max(min(XD, cols), 1);
    YD = max(min(YD, rows), 1);

    % Perform backward mapping and bilinear interpolation
    for c = 1:ch
        Z = double(input_img(:, :, c));
        
        output_img(:, :, c) = interp2(X, Y, Z, XD, YD, 'linear', 0);
    end

    % Display the warped image
    figure;
    imshow(output_img);
    title('Warped Image');
    pause(0.5);
end
