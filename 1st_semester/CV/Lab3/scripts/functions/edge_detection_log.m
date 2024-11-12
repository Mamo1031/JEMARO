% Edge detection based on LoG and zero-crossing
function edges = edge_detection_log(img, sd, threshold)
    LoG_filter = fspecial('log', 2 * ceil(3 * sd) + 1, sd);
    filtered_img = imfilter(img, LoG_filter, 'replicate');
    
    % Detect zero-crossings
    [rows, cols] = size(filtered_img);
    edges = zeros(rows, cols);
    for i = 2:rows-1
        for j = 2:cols-1
            if (filtered_img(i, j) * filtered_img(i+1, j) < 0 || ...
                filtered_img(i, j) * filtered_img(i, j+1) < 0) && ...
                abs(filtered_img(i, j) - filtered_img(i+1, j)) > threshold
                edges(i, j) = 1;
            end
        end
    end
end