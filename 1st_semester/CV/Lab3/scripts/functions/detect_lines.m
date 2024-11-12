% Apply Hough Transform for line detection on highway images
function detect_lines(img_path, num_peaks, nh_size)
    img = imread(img_path);
    img = im2double(rgb2gray(img));
    
    % Edge detection
    edges = edge(img, 'canny');
    
    % Perform Hough Transform
    [H, T, R] = hough(edges);
    P = houghpeaks(H, num_peaks, 'NHoodSize', nh_size);
    lines = houghlines(edges, T, R, P);
    
    % Display results
    figure, imshow(img), hold on;
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1), xy(:,2), 'LineWidth', 2, 'Color', 'green');
        % Plot beginnings and ends of lines
        plot(xy(1,1), xy(1,2), 'x', 'LineWidth', 2, 'Color', 'yellow');
        plot(xy(2,1), xy(2,2), 'x', 'LineWidth', 2, 'Color', 'red');
    end
    title(['Detected lines in ', img_path]);
    hold off;
end
