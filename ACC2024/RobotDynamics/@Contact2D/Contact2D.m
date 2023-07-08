classdef Contact2D
    
    properties

        % @type char
        Name

        % @type CoordinateFrame2D 
        % coordinate of the foot contact point
        CoordinateFrame

        % @type vector of char. For example ["x", "y", "theta"] means
        % all 3 dimensinos are constrained. ["y"] means only y is constrained. 
        ConstraintType


        % @type Force2D constraint force
        ConstraintForce

    end

    methods

        function obj = Contact2D(varargin)
            argin = struct(varargin{:});

            if isfield(argin, 'Name')
                assert(ischar(argin.Name), 'The name must be a character array.');
                obj.Name = argin.Name;
            end

            if isfield(argin, 'CoordinateFrame')
                obj.CoordinateFrame = argin.CoordinateFrame;
            end

            if isfield(argin, 'ConstraintType')
                obj.ConstraintType = argin.ConstraintType;
            end

            if isfield(argin, 'ConstraintForce')
                obj.ConstraintForce = argin.ConstraintForce;
            end
        end

        function contactPoint = getContactPoint(obj)
            contactPose = obj.CoordinateFrame.HomogeneousFromBase*[0;0;1];
            contactPoint = [];
            if ismember("x", obj.ConstraintType)
                contactPoint = [contactPoint; contactPose(1)];
            end

            if ismember("y", obj.ConstraintType)
                contactPoint = [contactPoint; contactPose(2)];
            end

            if ismember("theta", obj.ConstraintType)
                contactPoint = [contactPoint; contactPose(3)];
            end
        end

        function contactJacobianMatrix = getContactJacobian(obj, generalCoordinates)
            % general coordinates should be row vector
            % This function returns J(q) matrix in the J(q) \dot{q} = 0
            contactPoint = obj.getContactPoint();
            contactJacobianMatrix = jacobian(contactPoint, generalCoordinates);
        end

        function Jdot = getJdot(obj, generalCoordinates, generalVelocities)
            % find Jdot 
            % J(q) \ddot{q} + Jdot \dot{q} = 0
            J = obj.getContactJacobian(generalCoordinates);
            Jdot = J;
            for i = 1:size(J, 1)
                for j = 1:size(J, 2)
                    Jdot(i, j) = jacobian(J(i,j),generalCoordinates) * generalVelocities.';
                end
            end
        end


    end


end