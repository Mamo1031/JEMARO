function sharpening_filter_fft(filter_size, output_size)
    sharpen_filter = -1 * ones(filter_size);
    sharpen_filter(ceil(filter_size/2), ceil(filter_size/2)) = filter_size^2 - 1;
    
    padded_filter = zeros(output_size);
    center_start = floor((output_size - filter_size) / 2) + 1;
    center_end = center_start + filter_size - 1;
    
    padded_filter(center_start:center_end, center_start:center_end) = sharpen_filter;

    fft_sharpen = fftshift(fft2(padded_filter));
    
    magnitude = log(abs(fft_sharpen) + 1);
    
    figure;
    subplot(1, 2, 1);
    imagesc(padded_filter);
    axis equal;
    ylim([1, output_size+1]);
    colorbar;
    title('Padded Sharpening Filter');
    
    subplot(1, 2, 2);
    imshow(magnitude, []);
    title('FFT Magnitude of Sharpening Filter');
end
