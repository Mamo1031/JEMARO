% Slide 45 
function ex3_slide45_DetailSharpFilter(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);
    end
    image = double(image);
    
    % Step 1: smoothing 
    boxFilter5 = ones(5,5)/25;
    smoothedImage = conv2(image, boxFilter5, 'same');
    
    % Step 22: Detail-> OG - smoothed = detail
    detail = image - smoothedImage;
    
    % Step 3: Sharpen-> OG + detail = sharpen
    sharpenedImage = image + detail;
    
    figure;
    %OG
    subplot(2, 2, 1), imshow(uint8(image)), title('Original Image');
    %Smooth
    subplot(2, 2, 2), imshow(uint8(smoothedImage)), title('Smoothed Image');
    %detail
    subplot(2, 2, 3), imshow(uint8(detail)), title('Detail Image');
    %sharp
    subplot(2, 2, 4), imshow(uint8(sharpenedImage)), title('Sharpened Image');
    sgtitle('Slide 45 - Sharpening Process');
    
    set(gcf, 'Color', 'w'); %background color to white
end