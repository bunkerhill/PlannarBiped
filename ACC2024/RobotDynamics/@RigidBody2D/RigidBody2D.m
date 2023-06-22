classdef RigidBody2D

    properties

        % @type char
        Name

        % @type CoordinateFrame2D
        CoordinateFrame
        
        % @type double
        Mass

        % @type Point2D
        CenterOfMassInBodyFrame
        
        % @type double
        % inertia around z-axis at COM
        MomentOfInertia
    end

    methods

        function obj = RigidBody2D(varargin)
            argin = struct(varargin{:});

            if isfield(argin, 'Name')
                assert(ischar(argin.Name), 'The name must be a character array.');
                obj.Name = argin.Name;
            end

            if isfield(argin, 'CoordinateFrame')
                obj.CoordinateFrame = argin.CoordinateFrame;
            end

            if isfield(argin, 'Mass')
                obj.Mass = argin.Mass;
            end

            if isfield(argin, 'CenterOfMassInBodyFrame')
                obj.CenterOfMassInBodyFrame = argin.CenterOfMassInBodyFrame;
            end

            if isfield(argin, 'MomentOfInertia')
                obj.MomentOfInertia = argin.MomentOfInertia;
            end

        end
        
        % [x,y,theta]
        function position = forwardKinematics(obj, pointInBodyFrame)
            position = obj.CoordinateFrame.HomogeneousFromBase * [pointInBodyFrame.ColumnVector; 1];
            position = [position; obj.CoordinateFrame.RotationFromBase.Theta];
        end
        
        function angle = getCOMAngle(obj)
            angle = obj.CoordinateFrame.RotationFromBase.Theta;
        end

        function position = getCOMPosition(obj)
            position = obj.CoordinateFrame.HomogeneousFromBase * [obj.CenterOfMassInBodyFrame.ColumnVector; 1];
            position = position(1:2);
        end

        function potentialEnerge = getPotentialEnerge(obj)
            comPosition = obj.getCOMPosition();
            comHeight = comPosition(2);
            potentialEnerge = obj.Mass * comHeight * sym('g');
        end

        function jacobianMatrix = getCOMJacobian(obj, generalCoordinates)
            % general coordinates should be a row vector
            % returns partial COM position partial general coordinates
            % [ partial_x_partial_generalcoordinates; partial_y_partial_generalcoordinates]
            comPosition = obj.getCOMPosition();
            jacobianMatrix = jacobian(comPosition, generalCoordinates);
        end

        function comVelocity = getCOMVelocity(obj, generalCoordinates, generalVelocities)
            % general coordinates and general velocities should be row
            % vectors of the same length
            % return com velocity (column vector 2D)
            jacobianMatrix = obj.getCOMJacobian(generalCoordinates);
            comVelocity = jacobianMatrix * generalVelocities.';
        end

        function comTransitionEnergy = getCOMTransitionKineticEnergy(obj, generalCoordinates, generalVelocities)
            comVelocity = obj.getCOMVelocity(generalCoordinates, generalVelocities);
            comTransitionEnergy = 1/2*obj.Mass*(comVelocity.'*comVelocity);
        end

        function rotationEnergy = getRotationEnergy(obj, generalCoordinates, generalVelocities)
            rotationJacobian = jacobian(obj.CoordinateFrame.RotationFromBase.Theta, generalCoordinates);
            rotationVelocity = rotationJacobian*generalVelocities.';
            rotationEnergy = 1/2* obj.MomentOfInertia *(rotationVelocity*rotationVelocity);
        end

        function kineticEnergy = getKineticEnergy(obj, generalCoordinates, generalVelocities)
            kineticEnergy = obj.getCOMTransitionKineticEnergy(generalCoordinates, generalVelocities)...
                + obj.getRotationEnergy(generalCoordinates, generalVelocities);
        end
    end

    




end