% exercise 1 - Gaussian Noise and salt and pepper
function ex1(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);
    end

   
    figure('Position', [100, 100, 1200, 600]); 
    % OG image
    subplot(2, 3, 1), imagesc(double(image)), colormap gray, title('Original Image'); axis image; axis off;

    % Gaussian noise
    GaussianNoise = double(image) + 20 * randn(size(image));

    % GN image
    subplot(2, 3, 2), imagesc(GaussianNoise), colormap gray, title('Image with Gaussian Noise - std=20%'); axis image; axis off;

    % Salt & Pepper noise
    image = double(image);
    [rr, cc] = size(image);
    maxv = max(max(image(:)));
    indices = full(sprand(rr, cc, 0.2)); 
    mask1 = indices > 0 & indices < 0.5;  
    mask2 = indices >= 0.5; 
    out = image .* (~mask1);
    out = out .* (~mask2) + maxv * mask2;

    % Salt & Pepper noisy image
    subplot(2, 3, 3), imagesc(out), colormap gray, title('Salt & Pepper Noise (density = 20%)'); axis image; axis off;

    % Display histograms
    subplot(2, 3, 4), imhist(uint8(image)), title('Histogram of Original Image'); % Histogram of OG
    subplot(2, 3, 5), imhist(uint8(GaussianNoise)), title('Histogram of Gaussian Noise'); % Histogram of GN
    subplot(2, 3, 6), imhist(uint8(out)), title('Histogram of Salt & Pepper Noise'); % Histogram of S&P

    set(gcf, 'Color', 'w'); % background color to white -> to have the white backgorund for the report
end
