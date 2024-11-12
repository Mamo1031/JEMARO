function log_filter(sd)
    % Create Laplacian of Gaussian filter
    hsize = 2 * ceil(3 * sd) + 1; % Define filter size based on standard deviation
    LoG_filter = fspecial('log', hsize, sd);
    
    figure;
    imagesc(LoG_filter);
    colormap gray;
    title(['LoG Filter with Standard Deviation = ', num2str(sd)]);
end