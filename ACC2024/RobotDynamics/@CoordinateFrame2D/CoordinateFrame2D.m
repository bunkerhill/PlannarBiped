classdef CoordinateFrame2D
    properties
        % The name of coordinate frame
        % @type char
        Name

        % The parent frame of current frame
        % @type CoordinateFrame2D
        ParentFrame

        % @type Point2D
        Displacement

        % @type SO2, rotation from parent frame
        Rotation
        
        % @type matrix [Rotation, Displacement; 0, 0, 1], relative to
        % ParentFrame
        HomogeneousFromParent
        
        % @type matrix, homogeneous matrix from base coordinate frame
        HomogeneousFromBase
        
        % @type SO2, from Base Frame
        RotationFromBase
    end

    methods

        function obj = CoordinateFrame2D(varargin)
            argin = struct(varargin{:});
            
            if isfield(argin, 'Name')
                assert(ischar(argin.Name), 'The name must be a character array.');
                obj.Name = argin.Name;
                if strcmp(obj.Name, 'base')
                    obj.HomogeneousFromBase = eye(3);
                    obj.RotationFromBase = SO2(0);
                    return;
                end
            end

            if isfield(argin, 'ParentFrame')
               % parent frame can be empty for example the base frame has
               % no parent frame.
               obj.ParentFrame = argin.ParentFrame; 
            end

            if isfield(argin, 'Displacement')
                obj.Displacement = argin.Displacement;
            else
                obj.Displacement = Point2D('X', 0, 'Y', 0);
            end

            if isfield(argin, 'Rotation')
                obj.Rotation = argin.Rotation;
            else
                obj.Rotation = SO2(0);
            end

            obj.HomogeneousFromParent = [obj.Rotation.Matrix, obj.Displacement.ColumnVector;
                                      0, 0, 1];

            obj.HomogeneousFromBase = obj.ParentFrame.HomogeneousFromBase * obj.HomogeneousFromParent;

            obj.RotationFromBase = SO2(obj.ParentFrame.RotationFromBase.Theta + obj.Rotation.Theta);
        end

    end





end