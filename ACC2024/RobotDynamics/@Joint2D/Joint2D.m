classdef Joint2D
    % 2D joint that could either be revolute or prismatic

    properties (SetAccess=protected, GetAccess=public)
        % @type char
        Name


        % @type char
        Type

        % @type char
        ChildLink

        % @type char
        ParentLink

        % @type struct
        Limit

        % @type struct
        Actuator
    end

    methods

        function obj = Joint2D(varargin)
            % class constructor
            % Parameters:
        end
    end

end