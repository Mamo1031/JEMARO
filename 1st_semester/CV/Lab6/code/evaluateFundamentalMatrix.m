function evaluateFundamentalMatrix(F, P1, P2, I1, I2)
    % 1. Epipolar constraint check
    residuals = abs(sum(P2 .* (F * P1), 1));
    fprintf('Mean residual for epipolar constraint: %.6f\n', mean(residuals));

    % 2. Visualize epipolar lines
    visualizeEpipolarLines(I1, I2, F, P1(1:2, :)', P2(1:2, :)', 3);

    % 3. Compute and display epipoles
    [U, ~, V] = svd(F);
    epipole1 = V(:, end) / V(end, end); % Left epipole
    epipole2 = U(:, end) / U(end, end); % Right epipole

    fprintf('Left epipole: [%.3f, %.3f, %.3f]\n', epipole1);
    fprintf('Right epipole: [%.3f, %.3f, %.3f]\n', epipole2);
end
