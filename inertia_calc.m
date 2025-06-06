function M = inertia_calc(vec)
    % createMatrix - Generate a 3x3 matrix based on input vector [x, y, z]
    %
    % Syntax:  
    %     M = createMatrix([x, y, z])
    %
    % Inputs:
    %     vec - A 3-element vector containing [x, y, z] values
    %
    % Outputs:
    %     M - A 3x3 matrix structured as:
    %         [y^2 + z^2    -x*y       -x*z
    %          -x*y         x^2 + z^2   -y*z
    %          -x*z         -y*z        x^2 + y^2]
    %
    % Example: 
    %     M = createMatrix([1, 2, 3])
    
    % Extract components from input vector
    if length(vec) ~= 3
        error('Input must be a 3-element vector [x, y, z]');
    end
    
    x = vec(1);
    y = vec(2);
    z = vec(3);
    
    % Construct the 3x3 matrix
    M = [y^2 + z^2, -x*y, -x*z;
         -x*y, x^2 + z^2, -y*z;
         -x*z, -y*z, x^2 + y^2];
end