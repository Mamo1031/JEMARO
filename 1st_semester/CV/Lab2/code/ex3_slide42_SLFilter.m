% Slide 42 - Shift Left by 1 Pixel
function ex3_slide42_SLFilter(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);
    end
    image = double(image);
    
    % 7x7 - 1 in left center, 0 rest
    SLFilter = zeros(7,7);
    SLFilter(4,3) = 1;
    filteredSL = conv2(image, SLFilter, 'same');
    
    % imagesc()
    figure;
    subplot(2, 2, 1);
    imagesc(SLFilter);
    colormap gray;
    title('Shift-Left Filter');
    axis image;
    colorbar;
    
    % surf()
    subplot(2,2,2);
    surf(SLFilter);
    title('3D Surface Plot');
    shading interp;

    % OG
    subplot(2, 2, 3); 
    imshow(uint8(image));
    title('Original Image');
    
    % Filtered
    subplot(2, 2, 4);
    imshow(uint8(filteredSL));
    colormap gray;
    title('Filtered -> Shifted Left by 1 Pixel');
    
    sgtitle('Slide 42 - Shift-Left by 1 Pixel');

    set(gcf, 'Color', 'w'); % background color to white
end