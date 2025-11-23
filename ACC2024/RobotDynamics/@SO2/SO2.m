classdef SO2
    % special orthognal group (2D rotation)
    properties
        
        Theta
        
        % rotation matrix 
        Matrix
    end

    methods
        function obj = SO2(theta_in)
            obj.Theta = -theta_in;
            obj.Matrix = [cos(obj.Theta), -sin(obj.Theta);
                      sin(obj.Theta), cos(obj.Theta)];
        end

    end


end