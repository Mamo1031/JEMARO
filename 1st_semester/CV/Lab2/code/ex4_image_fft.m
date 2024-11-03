function image_fft(image_path)
    img = imread(image_path);
    
    fft_img = fftshift(fft2(double(img)));
    
    magnitude = log(abs(fft_img) + 1);
    
    figure;
    subplot(1, 2, 1);
    imshow(img, []);
    title('Original Image');
    
    subplot(1, 2, 2);
    imshow(magnitude, []);
    title('FFT Magnitude (log scale)');
end
