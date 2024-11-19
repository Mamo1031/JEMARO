function harris_corner_detector(I)
    % input:
    %   I - Grayscale image（double）
    
    % Calculate the x and y derivatives
    dx = [1 0 -1; 2 0 -2; 1 0 -1];
    dy = [1 2 1; 0 0 0; -1 -2 -1];
    Ix = conv2(I, dx, 'same');
    Iy = conv2(I, dy, 'same');
    
    % Show the x-derivative
    figure;
    imagesc(Ix);
    colormap gray;
    title('Ix');
    
    % Show the y-derivative
    figure;
    imagesc(Iy);
    colormap gray;
    title('Iy');
    
    % Calculate the product of the derivatives at each pixel
    Ix2 = Ix.^2;
    Iy2 = Iy.^2;
    Ixy = Ix .* Iy;

    % Calculate the sum of the products of the derivatives at each pixel using a Gaussian filter
    g = fspecial('gaussian', 9, 1.2);

    % Show Gaussian Filter
    figure;
    imagesc(g);
    colormap gray;
    title('Gaussian Filter');

    Sx2 = conv2(Ix2, g, 'same');
    Sy2 = conv2(Iy2, g, 'same');
    Sxy = conv2(Ixy, g, 'same');

    % Calculate the R value for each pixel
    k = 0.05;
    detM = Sx2 .* Sy2 - Sxy .^ 2;
    traceM = Sx2 + Sy2;
    R_map = detM - k * (traceM) .^ 2;

    % View R score map
    figure;
    imagesc(R_map);
    colormap jet;
    colorbar;
    title('R Score Map');

    % Get the maximum value of the R map
    M_max = max(R_map(:));

    % Detect corner regions by setting a threshold
    threshold = 0.3 * M_max;
    corner_reg = R_map > threshold;
    
    % Show Corner Regions
    figure;
    imagesc(corner_reg .* I);
    colormap gray;
    title('Corner Regions');

    pause(2);

    % Get the center of gravity of the corner region
    stats = regionprops(corner_reg, 'Centroid');
    if ~isempty(stats)
        centroids = cat(1, stats.Centroid);
    else
        centroids = [];
    end

    % Overlay detected corners on image
    figure;
    imshow(uint8(I));
    hold on;
    if ~isempty(centroids)
        plot(centroids(:,1), centroids(:,2), 'r*');
    end
    title('Detected Corners Overlapped on the Image');
end
