% Load MNIST data
load('../../data/mnist.mat');

% Specify classes
disp('Enter 2 class labels to include (e.g., [1, 7]):');
classes = input('Classes: ');
if numel(classes) ~= 2
    error('You must specify 2 class labels.');
end

% Specify the number of hidden units
nh = input('Number of hidden units: ');
if nh <= 0
    error('Number of hidden units must be positive.');
end

% Specify the number of epochs
epochs = input('Number of epochs: ');
if epochs <= 0
    error('Number of epochs must be positive.');
end

% Select the specified classes
selectedIdx = ismember(label, classes);
data = data(:, selectedIdx);
labels = label(selectedIdx);

% Normalize the data to [0, 1]
data = double(data) / 255;

% Map original class labels to continuous integers for compatibility with plotcl
[uniqueClasses, ~, newLabels] = unique(labels);

% Train the autoencoder
autoencoder = trainAutoencoder(data, nh, ...
    'EncoderTransferFunction', 'logsig', ...
    'DecoderTransferFunction', 'purelin', ...
    'MaxEpochs', epochs);

% Encode the data using the trained autoencoder
encodedData = encode(autoencoder, data);

% Transpose data to match plotcl.m convention
encodedDataT = encodedData';

% Plot the encoded data
figure;
plotcl(encodedDataT, newLabels, 1:2); % Plot the first two principal coordinates

% Create the legend dynamically based on the unique class labels
legendEntries = arrayfun(@(x) sprintf('%d', x), uniqueClasses, 'UniformOutput', false);
legend(legendEntries, 'Location', 'best');

% Add title and axis labels
% title('Encoded Data');
title(sprintf('Encoded Data (Hidden Units: %d, Epochs: %d)', nh, epochs));
xlabel('Feature 1');
ylabel('Feature 2');

% Save the plot to the current directory
outputFileName = sprintf('encoded_data_classes_%s.png', strjoin(string(classes), '_'));
saveas(gcf, outputFileName);
disp(['Plot saved as ', outputFileName]);
