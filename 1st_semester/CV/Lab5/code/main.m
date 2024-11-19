clc; clear; close all;

% NCC-based segmentation
images = {'../images/ur_c_s_03a_01_L_0376.png'; 
    '../images/ur_c_s_03a_01_L_0377.png'; 
    '../images/ur_c_s_03a_01_L_0378.png';
    '../images/ur_c_s_03a_01_L_0379.png';
    '../images/ur_c_s_03a_01_L_0380.png';
    '../images/ur_c_s_03a_01_L_0381.png'
};

% Create Red Car Template
red_template_pos = [689, 360, 80, 57]; % [x, y, width, height]
template_red = createTemplate(images{1}, red_template_pos);
% Show Red Car Template
figure, imshow(template_red);
title('Red Car Template');
% Red Car Detection
detectCar(images, template_red, 'Red Car');


% Create Dark Car Template - Normal
black_template_pos = [555, 362, 85, 51]; % [x, y, width, height]
template_black = createTemplate(images{1}, black_template_pos);
% Show Dark Car Template - Normal
figure, imshow(template_black);
title('Dark Car Template - Normal Size');
% Dark Car Detection
detectCar(images, template_black, 'Dark Car - Normal');

% Create Dark Car Template - Small
black_template_small_pos = [576 , 375, 60, 30]; 
template_black_small = createTemplate(images{1}, black_template_small_pos);
% Show Dark Car Template - Small
figure, imshow(template_black_small);
title('Dark Car Template - Small Size');
% Dark Car Detection
detectCar(images, template_black_small, 'Dark Car - Small');

% Create Dark Car Template - Large
black_template_large_pos = [534, 349, 160, 80];
template_black_large = createTemplate(images{1}, black_template_large_pos);
% Show Dark Car Template - Large
figure, imshow(template_black_large);
title('Dark Car Template - Large Size');
% Dark Car Detection
detectCar(images, template_black_large, 'Dark Car - Large');



% Harris corner detection
image_i235_origin = imread('../images/i235.png');
if size(image_i235_origin, 3) == 3
    image_i235_gray = double(rgb2gray(image_i235_origin));
else
    image_i235_gray = double(image_i235_origin);
end

harris_corner_detector(image_i235_gray);
