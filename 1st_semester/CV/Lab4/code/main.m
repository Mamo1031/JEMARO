image_files = {'../Lab4_testimages/ur_c_s_03a_01_L_0376.png'
                '../Lab4_testimages/ur_c_s_03a_01_L_0377.png'
                '../Lab4_testimages/ur_c_s_03a_01_L_0378.png'
                '../Lab4_testimages/ur_c_s_03a_01_L_0379.png'
                '../Lab4_testimages/ur_c_s_03a_01_L_0380.png'
                '../Lab4_testimages/ur_c_s_03a_01_L_0381.png'};

% Display grayscale images and split into RGB and HSV channels
display_images(image_files);

% Check the variation of the RGB components and of the Hue one
check_variation(image_files);

% Segment the dark car
segment_dark_car(image_files);

% Segment the red car
segment_red_car(image_files);
 
% Perform blob detection on the highlighted sunflowers
blob_detection();


