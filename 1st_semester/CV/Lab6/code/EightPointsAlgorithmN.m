function [F] = EightPointsAlgorithmN(P1, P2)
    % Normalize the points using the provided normalise2dpts function
    [nP1, T1] = normalise2dpts(P1); % Normalize P1
    [nP2, T2] = normalise2dpts(P2); % Normalize P2

    % Call the EightPointsAlgorithm function on the normalized points
    F_normalized = EightPointsAlgorithm(nP1, nP2);

    % Denormalize the resulting fundamental matrix
    F = T2' * F_normalized * T1;
end
