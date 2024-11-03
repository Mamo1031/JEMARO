% Slide 44 - Sharpening
function ex3_slide44_SharpFilter(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);  
    end
    image = double(image);

    % 7x7
    sharpFilter = zeros(7,7); 
    sharpFilter(4,4) = 2; % 2 in center
    boxFilter = ones(7,7) / 49; % avg
    sub = sharpFilter - boxFilter; % 2-avg
    filteredSharp = conv2(image, sub, 'same');

    % imagesc()
    figure;
    subplot(2,2,1);
    imagesc(sharpFilter);
    colormap gray;
    title('Sharpening Filter');
    axis image;
    colorbar;
    
    % surf()
    subplot(2,2,2);
    surf(sharpFilter);
    title('3D Surface Plot');
    shading interp;
    
    % OG
    subplot(2, 2, 3); 
    imshow(uint8(image));
    title('Original Image');
    
    % filtered
    subplot(2,2,4);
    imshow(uint8(filteredSharp));
    title('Filtered Image - Sharpening');
    
    sgtitle('Slide 44 - Sharpening Filter');

    set(gcf, 'Color', 'w'); % background color to white
end

