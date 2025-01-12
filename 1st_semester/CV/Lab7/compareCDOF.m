function [] = compareCDOF(videoFile, tau1, alpha, tau2, W) 
% This function compares the output of the change detection algorithm based
% on a running average, and of the optical flow estimated with the
% Lucas-Kanade algorithm.
% You must visualize the original video, the background and binary map
% obtained with the change detection, the magnitude and direction of the
% optical flow.
% tau1 is the threshold for the change detection
% alpha is the parameter to weight the contribution of current image and
% previous background in the running average
% tau2 is the threshold for the image differencing in the running average
% W is the side of the square patch to compute the optical flow

% Create a VideoReader object
videoReader = VideoReader(videoFile);

% Initialize background and optical flow
background = [];  % will set with first frame
opticalFlowObj = opticalFlowLK('NoiseThreshold',0.003,'WindowSize',W);

frameCount = 0;
figure('Name','compareCDOF','NumberTitle','off'); % create figure

firstFrame = true; % flag for first flame

% Loop through each frame of the video
while hasFrame(videoReader)
    % Read the next frame
    frame = readFrame(videoReader);

    % Convert to grayscale (uint8) then to double
    grayFrame = rgb2gray(frame);
    currFrame = im2double(grayFrame);

    % Initialization for first flame
    if firstFrame
        background = currFrame; 
        firstFrame = false;    
        continue;        
    end

    % Change detection using running average
    diff = abs(currFrame - background); % Difference with background

    % Update background only where difference < tau1
    mask = (diff < tau1);
    % Propose updated background
    updatedBg = alpha * currFrame + (1 - alpha) * background;
    % Only update in regions where mask=1
    background = mask .* updatedBg + (~mask) .* background;

    % Create binary map where difference > tau2
    binaryMap = diff > tau2;

    % Optical flow calculation
    flow = estimateFlow(opticalFlowObj, currFrame);
    flowImage = convertToMagDir(flow.Vx, flow.Vy); 

    % (1) Original color frame
    subplot(2, 2, 1);
    imshow(frame, 'Border', 'tight');
    title(sprintf('Original (Frame #%d)', frameCount));

    % (2) Optical flow map
    subplot(2, 2, 2);
    imshow(flowImage, 'Border', 'tight');
    title('Optical Flow');

    % (3) Running average (convert background [0..1] -> uint8 for display)
    subplot(2, 2, 3);
    bgForDisplay = im2uint8(background);
    imshow(bgForDisplay, 'Border', 'tight');
    title('Running Average (Background)');

    % (4) Binary map from change detection
    subplot(2, 2, 4);
    imshow(binaryMap, 'Border', 'tight');
    title('Binary Map');

    drawnow;
    frameCount = frameCount + 1;
end

fprintf('Finished displaying video: %s\n', videoFile);
end