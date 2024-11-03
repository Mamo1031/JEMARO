function filtered_img = applyMovingAverageFilter(image, filter_size)
    avg_filter = fspecial('average', filter_size);
    filtered_img = imfilter(image, avg_filter, 'replicate');
    [x, y] = meshgrid(0.5:filter_size-0.5, 0.5:filter_size-0.5);
    
    figure;
    subplot(1, 2, 1);
    imagesc(x(1,:), y(:,1), avg_filter);
    colorbar;
    axis equal;
    xlim([0, filter_size]);
    ylim([0, filter_size]);
    xticks(0:1:filter_size - 1);
    yticks(0:1:filter_size - 1);
    title(['Moving Average Filter ', num2str(filter_size), 'x', num2str(filter_size)]);
    subplot(1, 2, 2);
    surf(avg_filter);
    axis equal;
    xlim([0, filter_size+1]);
    ylim([0, filter_size+1]);
    zlim([-1, max(avg_filter(:))]);
    xticks(0:1:filter_size);
    yticks(0:1:filter_size);
    zticks(-1:1:max(avg_filter(:)));
    title('Filter Surface');

    figure;
    subplot(1, 2, 1);
    imshow(filtered_img);
    title(['Filtered Image (Avg) ', num2str(filter_size), 'x', num2str(filter_size)]);
    subplot(1, 2, 2);
    histogram(filtered_img);
    title('Histogram of Filtered Image');
end