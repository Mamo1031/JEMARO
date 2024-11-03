% Slide 43 - Blurred with a Box Filter (7x7)
function ex3_slide43_BBFilter(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);
    end
    image = double(image);

    % 7x7 - avg
    boxFilter = ones(7,7) / 49;
    filteredBox = conv2(image, boxFilter, 'same');
    
    % imagesc()
    figure;
    subplot(2, 2, 1);
    imagesc(boxFilter);
    colormap gray;
    title('Blurring Box Filter');
    axis image;
    colorbar;
    
    % surf()
    subplot(2,2,2);
    surf(boxFilter);
    title('3D Surface Plot');
    shading interp;

    % OG
    subplot(2, 2, 3); 
    imshow(uint8(image));
    title('Original Image');
    
    % filtered
    subplot(2, 2, 4);
    imshow(uint8(filteredBox));
    colormap gray;
    title('Filtered -> Blurring');
    
    sgtitle('Slide 43 - Blurred with Box Filter');

    set(gcf, 'Color', 'w'); %background color to white
end