classdef Force2D
    properties

        % @type double or symble 
        X

        % @type double or symble
        Y
        % @type Matlab column vector
        ColumnVector
    end

    methods
        function obj = Force2D(varargin)
            argin = struct(varargin{:});
            if isfield(argin, 'X')
                obj.X = argin.X;
            else
                obj.X = 0;
            end

            if isfield(argin, 'Y')
                obj.Y = argin.Y;
            else
                obj.Y = 0;
            end

            obj.ColumnVector = [obj.X ; obj.Y];
        end
        
    end


end