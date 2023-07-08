classdef GeneralizedForce2D
    % Appendix B.4.6 Generalized Forces and Torques of Feedback Control of
    % Dynamic Bipedal Robot Locomotion 
    
    properties
        % @type char
        Name

        % @type Force2D
        % The external force applied on body
        Force

        % @type Point2D
        % The point where external force is applied
        Displacement

        % @type double or symbole
        Torque

        % @type SO2
        % The torque is applied on this angle
        Angle

    end
    
    methods
        function obj = GeneralizedForce2D(varargin)
            argin = struct(varargin{:});

            if isfield(argin, 'Name')
                assert(ischar(argin.Name), 'The name must be a character array.');
                obj.Name = argin.Name;
            end

            if isfield(argin, 'Force')
                obj.Force = argin.Force;
            else
                obj.Force = Force2D();
            end

            if isfield(argin, 'Displacement')
                obj.Displacement = argin.Displacement;
            else
                obj.Displacement = Point2D();
            end

            if isfield(argin, 'Torque')
                obj.Torque = argin.Torque;
            else
                obj.Torque = 0;
            end

            if isfield(argin, 'Angle')
                obj.Angle = argin.Angle;
            else
                obj.Angle = SO2(0);
            end

        end


        function virtualWork = getVirtualWork(obj)
            virtualWork = obj.Force.ColumnVector.' * obj.Displacement.ColumnVector + obj.Torque * obj.Angle.Theta;
        end

        function generalizedForce = getGeneralizedForce(obj, generalizedCoordinates)
            virtualWork = obj.getVirtualWork();
            generalizedForce = jacobian(virtualWork, generalizedCoordinates).';
        end
    end
end

