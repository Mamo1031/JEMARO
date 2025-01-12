function [rgbImage] = convertToMagDir(u, v)
% This function produces a visualization of the optical flow based on
% magnitude and direction of each vector
% u and v are the matrices with the two components of the optical flow

% Calcolo della magnitudine e della direzione del flusso ottico
magnitude = sqrt(u.^2 + v.^2); % Magnitudine
direction = atan2(v, u);       % Direzione in radianti

% Normalizzazione per visualizzazione
magnitudeNorm = magnitude / max(magnitude(:)); % Normalizza la magnitudine

% Conversione da direzione e magnitudine a spazio colore HSV
hue = (direction + pi) / (2 * pi); % Direzione normalizzata in [0, 1]
saturation = ones(size(hue));      % Saturazione costante
value = magnitudeNorm;             % Intensità basata sulla magnitudine

% Conversione da HSV a RGB
hsvImage = cat(3, hue, saturation, value);
rgbImage = hsv2rgb(hsvImage);