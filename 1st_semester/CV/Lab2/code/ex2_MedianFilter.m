function filtered_img = applyMedianFilter(image, filter_size)
    filtered_img = medfilt2(image, [filter_size filter_size]);
    
    figure;
    subplot(1, 2, 1);
    imshow(uint8(filtered_img));
    title(['Filtered Image (Median) ', num2str(filter_size), 'x', num2str(filter_size)]);
    subplot(1, 2, 2);
    histogram(uint8(filtered_img));
    title('Histogram of Filtered Image');
end