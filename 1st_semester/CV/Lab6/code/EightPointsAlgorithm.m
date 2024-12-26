function [F] = EightPointsAlgorithm(P1, P2)

    % 1. Write down the matrix A (see the slides...)
    n = size(P1,2);
    A = zeros(n, 9);
 
    for i=1:n
        x1 = P1(1,i);
        y1 = P1(2,i);
        x2 = P2(1,i);
        y2 = P2(2,i);
        A(i,:) = [x1*x2, x1*y2, x1, y1*x2, y1*y2, y1, x2, y2, 1];  
    end
 
    % 2. Compute the SVD decomposition of A 
    [U, D, V]=svd(A);
    
    % and select as solution f the last column of V.
    f = V(:,end);

    
    % Reshape the column vector f so to obtain a matrix F (see function reshape)
    F = reshape(f,[3,3])';

    % Force the rank of F to be 2:
    % Use again the SVD to decompose the matrix 
    [U, D, V] = svd(F);

    % Set D(3,3)=0
    D(3,3) = 0;

    % Recompute the final F: F=U*D*VT.
    F = U * D * V';
end


