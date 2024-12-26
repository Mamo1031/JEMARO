function [P1, P2] = matchFeaturesBetweenImages(I1, I2)
    % Convert images to grayscale if needed
    if size(I1, 3) == 3
        I1 = rgb2gray(I1);
    end
    if size(I2, 3) == 3
        I2 = rgb2gray(I2);
    end

    % Detect and extract features
    points1 = detectSURFFeatures(I1);
    points2 = detectSURFFeatures(I2);

    [features1, validPoints1] = extractFeatures(I1, points1);
    [features2, validPoints2] = extractFeatures(I2, points2);

    % Match features
    indexPairs = matchFeatures(features1, features2);

    % Retrieve matched points
    matchedPoints1 = validPoints1(indexPairs(:, 1), :).Location;
    matchedPoints2 = validPoints2(indexPairs(:, 2), :).Location;

    % Convert to homogeneous coordinates
    P1 = [matchedPoints1'; ones(1, size(matchedPoints1, 1))];
    P2 = [matchedPoints2'; ones(1, size(matchedPoints2, 1))];
end
