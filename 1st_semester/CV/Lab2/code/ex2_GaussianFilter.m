function filtered_img = applyGaussianFilter(image, filter_size)
    gauss_filter = fspecial('gaussian', filter_size, filter_size/3);
    filtered_img = imfilter(image, gauss_filter, 'replicate');
    [x, y] = meshgrid(0.5:filter_size-0.5, 0.5:filter_size-0.5);

    figure;
    subplot(1, 2, 1);
    imagesc(x(1,:), y(:,1), gauss_filter);
    colorbar;
    axis equal;
    xlim([0, filter_size]);
    ylim([0, filter_size]);
    xticks(0:1:filter_size - 1);
    yticks(0:1:filter_size - 1);
    title(['Gaussian Filter ', num2str(filter_size), 'x', num2str(filter_size)]);
    subplot(1, 2, 2);
    surf(gauss_filter);
    xlim([0, filter_size+1]);
    ylim([0, filter_size+1]);
    xticks(0:1:filter_size);
    yticks(0:1:filter_size);
    title('Filter Surface');

    figure;
    subplot(1, 2, 1);
    imshow(uint8(filtered_img));
    title(['Filtered Image (Gaussian) ', num2str(filter_size), 'x', num2str(filter_size)]);
    subplot(1, 2, 2);
    histogram(uint8(filtered_img));
    title('Histogram of Filtered Image');
end