function gaussian_filter_fft(filter_size, sigma)
    gauss_filter = fspecial('gaussian', filter_size, sigma);
    
    fft_gauss = fftshift(fft2(gauss_filter));
    
    magnitude = log(abs(fft_gauss) + 1);
    
    figure;
    subplot(1, 2, 1);
    imagesc(gauss_filter);
    axis equal;
    ylim([1, filter_size+1]);
    colorbar;
    title('Gaussian Filter');
    
    subplot(1, 2, 2);
    imshow(magnitude, []);
    title('FFT Magnitude of Gaussian Filter');
end
