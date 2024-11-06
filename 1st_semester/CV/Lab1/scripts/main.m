image_files = {'../Lab1_data/flower.jpg', '../Lab1_data/boccadasse.jpg'};
image_titles = {'flower', 'boccadasse'};

for img_idx = 1:length(image_files)
    image = imread(image_files{img_idx});
    
    figure(img_idx);
    imshow(image);
    title(['Original Image - ', image_titles{img_idx}]);
    pause(0.5);
    
    translated_image = translate_image(image, 30, 30);
    rotated_image = rotate_image(image, 30);
end

flower_image = imread(image_files{1});
flower_data = load('../Lab1_data/data_flower.mat');
flower_XD = flower_data.XD;
flower_YD = flower_data.YD;
warped_flower = warp_image(flower_image, flower_XD, flower_YD);

boccadasse_image = imread(image_files{2});
boccadasse_data = load('../Lab1_data/data_boccadasse.mat');
boccadasse_XD = boccadasse_data.XD;
boccadasse_YD = boccadasse_data.YD;
warped_boccadasse = warp_image(boccadasse_image, boccadasse_XD, boccadasse_YD);
