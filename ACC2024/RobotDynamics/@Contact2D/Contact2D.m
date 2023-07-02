classdef Contact2D
    
    properties

        % @type char
        Name

        % @type CoordinateFrame2D 
        % coordinate of the foot contact point
        CoordinateFrame

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

        end

        function contactPoint = getContactPoint(obj)
            contactPoint = [1, 0, 0; 0,1, 0]*obj.CoordinateFrame.HomogeneousFromBase*[0;0;1];
        end

        function contactJacobianMatrix = getContactJacobian(obj, generalCoordinates)
            % general coordinates should be row vector
            contactPoint = obj.getContactPoint();
            contactJacobianMatrix = jacobian(contactPoint, generalCoordinates);
        end


    end


end