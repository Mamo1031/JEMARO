function [] = compareCDAlgo(videoFile, tau1, alpha, tau2)
% This function compares the output of the change detection algorithm when
% using two possible background models:
% 1. A static model, e.g. a single frame or the average of N frames.
% In this case, the background is computed once and for all
% 2. A running average to update the model. In this case the background is
% updated, if needed, at each time instant
% You must visualize the original video, the background and binary map
% obtained with 1., the background and binary map
% obtained with 2.
% tau1 is the threshold for the change detection
% alpha is the parameter to weight the contribution of current image and
% previous background in the running average
% tau2 is the threshold for the image differencing in the running average

% Create a VideoReader object
videoReader = VideoReader(videoFile);

% 1. static model 
% initialize static background (average of the first N frames)
N = 10;                 % number of frames to calculate the static background
frameCount = 0;         % this is the count of frames that has passed
staticBackground = 0;   % saves the result of the static background

% loop to compute the average of the first N frames (static background)
% staticBackgroun = (1/N) * sum(I_i) for i=1 to N
while hasFrame(videoReader) && frameCount < N
    frame = rgb2gray(readFrame(videoReader));           % convert to grayscale
    staticBackground = staticBackground + double(frame);% sum the frames
    frameCount = frameCount + 1;                        % updates the count of frames
end
staticBackground = staticBackground / frameCount; % computes the final part of the average of frames
staticBackground = uint8(staticBackground);       % convert to uint8 for visualization


% 2. running average to update the model
% initialize running average model
videoReader.CurrentTime = 0;                            % reset the video to the 1st frame
runBackground = rgb2gray(readFrame(videoReader));  % use the 1st frame
runBackground = double(runBackground);        % convert to double before differencing

% Loop through each frame of the video
while hasFrame(videoReader)
    % Read the next frame
    frame = rgb2gray(readFrame(videoReader)); % convert to grayscale
    
    % Display the frame
    figure(1), subplot(2, 3, 1), imshow(frame, 'Border', 'tight');
    title(sprintf('Frame %d', round(videoReader.CurrentTime * videoReader.FrameRate)));
    
    % 1. static
    % Display the static background
    figure(1), subplot(2, 3, 2), imshow(staticBackground, 'Border', 'tight');
    title('Static Background');

    % compute the difference between the static background and the current frame and apply tau1
    % diffStatic = |I_t - staticBackground|
    diffStatic = abs(double(frame) - double(staticBackground));  % Difference between static background and frame
    binaryMap1 = diffStatic > tau1;  % apply tau1 to generate binary map: binary = 1 if diffStatic>tau1, else 0
    
    % Display the binary map obtained with the static background
    figure(1), subplot(2, 3, 3), imshow(binaryMap1, 'Border', 'tight');
    title('Binary map 1');
    
    % 2. running average
    % Display the running average 
    figure(1), subplot(2, 3, 5), imshow(uint8(runBackground), 'Border', 'tight');
    title('Running Average');

    % compute the difference between the running background and the current frame and apply tau2
    % diffAdaptive = |I_t - runBackground|
    diffAdaptive = abs(double(frame) - runBackground);  % difference between running background and frame
    binaryMap2 = diffAdaptive > tau2;  % apply tau2 to generate binary map: binary = 1 if diffAdaptive>tau2, else 0
    
    % Display the binary map obtained with the running average
    figure(1), subplot(2, 3, 6), imshow(binaryMap2, 'Border', 'tight');
    title('Binary Map 2 (Running)');
    
    % update the running background using the running average formula:
    % runningBackground_t = alpha * I_t + (1-alpha) *
    % runningBackground_(t_1)
    runBackground = alpha * double(frame) + (1 - alpha) * runBackground;
    
end

% Close the figure when playback is finished
close all;

fprintf('Finished processing video: %s\n', videoFile);
end
