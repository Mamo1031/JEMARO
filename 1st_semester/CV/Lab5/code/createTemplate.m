function template = createTemplate(imagePath, position)
% Function to create the template from an image
% position: [x, y, width, height] do template

    img = imread(imagePath);
    gray_img = im2gray(img);

    x = position(1);
    y = position(2);
    width = position(3);
    height = position(4);

    template = gray_img(y:y+height-1, x:x+width-1);
end
