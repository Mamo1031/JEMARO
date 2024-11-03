% main script
image_files = {'../image/input/tree.png', '../image/input/i235.png'};

for img_idx = 1:length(image_files)
    % Exercise 1 - Gaussian and Salt&Pepper Noise
    ex1(image_files{img_idx});
    
    
    % Exercise 2 - Remove the noise by using filters
    filter_sizes = [3, 7];
    image = imread(image_files{img_idx});
    
    for i = 1:length(filter_sizes)
        filter_size = filter_sizes(i);

        avg_filtered = ex2_MovingAverageFilter(image, filter_size);
        gauss_filtered = ex2_GaussianFilter(image, filter_size);
        median_filtered = ex2_MedianFilter(image, filter_size);
    end
    
    % Exercise 3 - Slides 41 - 45
    ex3_slide41_IDFilter(path2);
    ex3_slide42_SLFilter(path2);
    ex3_slide43_BBFilter(path2);
    ex3_slide44_SharpFilter(path2);
    ex3_slide45_DetailSharpFilter(path2);


    % Exercise 4 - Fourier Transform
    ex4_image_fft(image_files{img_idx});
end

ex4_gaussian_filter_fft(101, 5);
ex4_sharpening_filter_fft(7, 101);
