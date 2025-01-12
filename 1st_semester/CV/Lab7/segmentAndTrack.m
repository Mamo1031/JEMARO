function [] = segmentAndTrack(videoFile, tau1, alpha, tau2) 
% This function ...
% tau1 is the threshold for the change detection
% alpha is the parameter to weight the contribution of current image and
% previous background in the running average
% tau2 is the threshold for the image differencing in the running average
% Add here input parameters to control the tracking procedure if you need...

% Create a VideoReader object
videoReader = VideoReader(videoFile);

% Initialize background
background = [];

% Index for frames
i = 0;

% For storing the trajectory
trajectory = [];  % Nx2 matrix storing [x,y] of the target

% Flag to indicate if we already picked the target
targetPicked = false;

% Current estimate of target position
currentTarget = [NaN, NaN];

figure('Name','SegmentAndTrack','NumberTitle','off');

% Loop through each frame of the video
while hasFrame(videoReader)
    % Read the next frame
    frame = readFrame(videoReader);

    % Convert to grayscale -> double
    grayFrame = rgb2gray(frame);
    currFrame = im2double(grayFrame);

    % If background is empty, initialize with first frame
    if isempty(background)
        background = currFrame;
        continue
    end

    % Change detection with running average
    diffVal = abs(currFrame - background);

    % Update background only where diffVal < tau1
    % This prevents including large changes into the background
    mask = (diffVal < tau1);
    updatedBg = alpha * currFrame + (1 - alpha) * background;
    background = mask .* updatedBg + (~mask) .* background;

    % Binary map where diffVal > tau2
    binaryMap = diffVal > tau2;

    % Identify connected components
    CC = bwconncomp(binaryMap);
    stats = regionprops(CC, 'Centroid', 'Area');

    % Display the frame
    subplot(2,2,1);
    imshow(frame, 'Border', 'tight');
    title(sprintf('Frame %d', i));

    % Display the running average
    subplot(2,2,2);
    bgDisp = im2uint8(background);  % for visualization
    imshow(bgDisp, 'Border','tight');
    title('Running Average (Background)');

    % Display the binary map
    subplot(2,2,3);
    imshow(binaryMap, 'Border','tight');
    title('Binary Map');

    % Show the same frame with tracking results
    subplot(2,2,4);
    imshow(frame, 'Border','tight');
    hold on;

    % At some frame (e.g. i=1380), we pick a point on the target
    if (i == 1380 && ~targetPicked)
        % Pause and let user pick a point with the mouse
        disp('Pick the target with the mouse click.');
        [xPicked, yPicked] = ginput(1);
        currentTarget = [xPicked, yPicked];
        targetPicked = true;
        % Initialize trajectory
        trajectory = currentTarget; 
    end

    if targetPicked
        % We have the connected components stats
        % For each component, find the centroid, measure distance to currentTarget
        if ~isempty(stats)
            centroids = cat(1, stats.Centroid);  % Nx2
            % find the closest centroid
            distList = sqrt((centroids(:,1) - currentTarget(1)).^2 + ...
                            (centroids(:,2) - currentTarget(2)).^2);
            [minDist, idxMin] = min(distList);

            if minDist < 100  % some max distance to avoid jump
                currentTarget = centroids(idxMin,:);
            end
        end

        % add to trajectory
        trajectory = [trajectory; currentTarget];

        % draw a marker
        plot(currentTarget(1), currentTarget(2), 'ro', 'MarkerSize',8,'LineWidth',2);
        % draw trajectory
        plot(trajectory(:,1), trajectory(:,2), 'r-', 'LineWidth',1.5);
    end

    title('Tracking (Frame+Trajectory)');
    hold off;

    drawnow;

    i = i + 1;
end

% After finishing the video, show final trajectory on the last frame
fprintf('Finished displaying video: %s\n', videoFile);

if ~isempty(trajectory)
    disp('Final trajectory:');
    disp(trajectory);
end

fprintf('Finished displaying video: %s\n', videoFile);
end