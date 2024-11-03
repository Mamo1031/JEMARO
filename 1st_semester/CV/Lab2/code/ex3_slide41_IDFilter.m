% Slide 41 - No change Filter (id filter)
function ex3_slide41_IDFilter(path)
    image = imread(path);
    if size(image, 3) == 3
        image = rgb2gray(image);
    end
    image = double(image);

    % 7x7 filter - 1 in center, rest 0
    IDFilter = zeros(7,7);
    IDFilter(4,4) = 1;
    filteredID = conv2(image, IDFilter, 'same');
    
    % imagesc()
    figure;
    subplot(2, 2, 1);
    imagesc(IDFilter);
    colormap gray;
    title('Identity Filter');
    axis image;
    colorbar;
    
    % surf()
    subplot(2, 2, 2);
    surf(IDFilter);
    title('3D Surface Plot');
    shading interp;

    % OG
    subplot(2, 2, 3); 
    imshow(uint8(image));
    title('Original Image');
    
    % Filtered
    subplot(2, 2, 4);
    imshow(uint8(filteredID));
    colormap gray;
    title('Filtered -> No change');
    
    sgtitle('Slide 41 - No Change Filter')

    set(gcf, 'Color', 'w'); % background color to white

end
