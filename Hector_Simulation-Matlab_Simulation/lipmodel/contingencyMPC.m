classdef contingencyMPC

    properties
        % one step time
        delta
        
        % horizon time
        T_h

        vectorLength


        
        % foot half length
        footHalfLength

        % foot half width
        footHalfWidth
        
        % one step time
        gait_time
        single_support_time
        double_support_time
        
        % 9.8m/s^2
        g
        
        % com height
        L

        % timer
        tim

        distime

        currentTime
        previewTimeHorizon

        stanceConstraint
        ZMPConstraintXMin
        ZMPConstraintXMax
        ZMPConstraintYMin
        ZMPConstraintYMax

        ZMPXVelocity
        ZMPYVelocity
        previewZMPXPosition
        previewZMPYPosition

        % contingency parameters
        j_max
        j_min
        a_max
        a_min
        T_u
        T_l
        
    end

    methods
        function obj = contingencyMPC(comHeight, footHalfLength, footHalfWidth, ddxy_s_max_in, ddxy_s_min_in)
            obj.g = 9.8; %m/s^2
            obj.L = comHeight;
            obj.delta = 0.01;
            obj.T_h = 1.0;
            obj.vectorLength = round(obj.T_h/obj.delta);
            
            obj.footHalfLength = footHalfLength;
            obj.footHalfWidth = footHalfWidth;
            
            obj.single_support_time = 0.2;
            obj.double_support_time = 0.0;
            obj.gait_time = obj.single_support_time + obj.double_support_time;
            obj.tim = 0;
            obj.distime = 0.2;
            
            % contingency parameters
            obj.j_max = [1,2];
            obj.j_min = [-1,-2];
            obj.a_max = ddxy_s_max_in;
            obj.a_min = ddxy_s_min_in;
        end
        
        
        % solve the mpc problem
        function obj = MPC(obj,xi,p_z, currentTime, stanceConstraint,ddxy_s)
            
            obj.T_u(1) = (obj.a_max(1)-ddxy_s(1))/obj.j_max(1);
            obj.T_u(2) = (obj.a_max(2)-ddxy_s(2))/obj.j_max(2);
            obj.T_l(1) = (obj.a_min(1)-ddxy_s(1))/obj.j_min(1);
            obj.T_l(2) = (obj.a_min(2)-ddxy_s(2))/obj.j_min(2);

            obj.currentTime=currentTime;
            obj.stanceConstraint = stanceConstraint;
            obj.previewTimeHorizon=currentTime+(0:1:obj.vectorLength-1)*obj.delta;
            obj = obj.ZMPConstraintSampledFromFootPlacement();

            % objective function
            H = eye(2*obj.vectorLength);

            % equality constraint
            omega = sqrt(obj.g/obj.L);
            lamda = exp(-omega*obj.delta);
            b_T = zeros(1,obj.vectorLength);
            for i = 1:obj.vectorLength
                b_T(i) = lamda^(i-1);
            end
            Aeq1 = (1-lamda)/omega/(1-lamda^obj.vectorLength)*b_T;
            beq1 = xi(1)-p_z(1);

            Aeq2 = zeros(1,2*obj.vectorLength);
            Aeq2(1,1) = 1;
            Aeq2(1,1+obj.vectorLength) = -1;

            Aeq = [blkdiag(Aeq1,Aeq1);
                   Aeq2];
               
            beq = [beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_u(1))) + obj.a_max(1)*exp(-omega*obj.T_u(1)) ) - 1/(omega^3)*obj.j_max(1)*(1-(1+obj.T_u(1)*omega)*exp(-omega*obj.T_u(1)));
                   beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_l(1))) + obj.a_min(1)*exp(-omega*obj.T_l(1)) ) - 1/(omega^3)*obj.j_min(1)*(1-(1+obj.T_l(1)*omega)*exp(-omega*obj.T_l(1)));
                   0];

            % control constraint
            lb = []; 
            ub = [];
            % inequality constraint
            p = ones(obj.vectorLength,1);
            temp = ones(obj.vectorLength);
            P = tril(temp)*obj.delta;
            A1 = [P;
                  -P];
            A = blkdiag(A1,A1);

            b1 = [X_max-p*p_z(1);
                 -(X_min-p*p_z(1))];
            b = [b1;
                 b1];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            b = [obj.ZMPConstraintXMax-p*p_z(1);
                 -(obj.ZMPConstraintXMin-p*p_z(1));
                 obj.ZMPConstraintYMax-p*p_z(2);
                 -(obj.ZMPConstraintYMin-p*p_z(2))];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Xz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---X DIRECTION SOLUTION NOT FOUND---");
            end

            %----------------------------------------------------------------------------

            % objective function
            H = eye(2*obj.vectorLength);

            % equality constraint
            omega = sqrt(obj.g/obj.L);
            lamda = exp(-omega*obj.delta);
            b_T = zeros(1,obj.vectorLength);
            for i = 1:obj.vectorLength
                b_T(i) = lamda^(i-1);
            end
            Aeq1 = (1-lamda)/omega/(1-lamda^obj.vectorLength)*b_T;
            beq1 = xi(1)-p_z(1);

            Aeq2 = zeros(1,2*obj.vectorLength);
            Aeq2(1,1) = 1;
            Aeq2(1,1+obj.vectorLength) = -1;

            Aeq = [blkdiag(Aeq1,Aeq1);
                   Aeq2];
            beq = [beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_u(2))) + obj.a_max(2)*exp(-omega*obj.T_u(2)) ) - 1/(omega^3)*obj.j_max(2)*(1-(1+obj.T_u(2)*omega)*exp(-omega*obj.T_u(2)));
                   beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_l(2))) + obj.a_min(2)*exp(-omega*obj.T_l(2)) ) - 1/(omega^3)*obj.j_min(2)*(1-(1+obj.T_l(2)*omega)*exp(-omega*obj.T_l(2)));
                   0];
            % control constraint
            lb = [];
            ub = [];

            % inequality constraint
            p = ones(obj.vectorLength,1);
            temp = ones(obj.vectorLength);
            P = tril(temp)*obj.delta;
            A1 = [P;
                  -P];
            A = blkdiag(A1,A1);

            b1 = [X_max-p*p_z(1);
                 -(X_min-p*p_z(1))];
            b = [b1;
                 b1];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            b = [obj.ZMPConstraintXMax-p*p_z(1);
                 -(obj.ZMPConstraintXMin-p*p_z(1));
                 obj.ZMPConstraintYMax-p*p_z(2);
                 -(obj.ZMPConstraintYMin-p*p_z(2))];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Yz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---Y DIRECTION SOLUTION NOT FOUND---");
            end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.ZMPXVelocity = Pz_dot(1:obj.vectorLength);
            obj.ZMPYVelocity = Pz_dot(obj.vectorLength+1:end);

            obj.previewZMPXPosition = zeros(obj.vectorLength,1);
            obj.previewZMPYPosition = zeros(obj.vectorLength,1);
            obj.previewZMPXPosition(1)=p_z(1);
            obj.previewZMPYPosition(1)=p_z(2);
            for i=1:obj.vectorLength-1
                obj.previewZMPXPosition(i+1)=obj.previewZMPXPosition(i)+obj.delta*obj.ZMPXVelocity(i);
                obj.previewZMPYPosition(i+1)=obj.previewZMPYPosition(i)+obj.delta*obj.ZMPYVelocity(i);
            end
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end

        % Generate ZMP constraint
        function obj = ZMPConstraintSampledFromFootPlacement(obj)
            obj.ZMPConstraintXMax = zeros(obj.vectorLength, 1);
            obj.ZMPConstraintXMin = zeros(obj.vectorLength, 1);
            obj.ZMPConstraintYMax = zeros(obj.vectorLength, 1);
            obj.ZMPConstraintYMin = zeros(obj.vectorLength, 1);
            for i=1:obj.vectorLength
                stanceFootIndex=find(obj.previewTimeHorizon(i)>=obj.stanceConstraint.time,1,"last");
                obj.ZMPConstraintXMax(i) = obj.stanceConstraint.ankleX(stanceFootIndex)+obj.footHalfLength;
                obj.ZMPConstraintXMin(i) = obj.stanceConstraint.ankleX(stanceFootIndex)-obj.footHalfLength;
                obj.ZMPConstraintYMax(i) = obj.stanceConstraint.ankleY(stanceFootIndex)+obj.footHalfWidth;
                obj.ZMPConstraintYMin(i) = obj.stanceConstraint.ankleY(stanceFootIndex)-obj.footHalfWidth;
            end
        end


        function [] = drawZMPPreviewAndConstraint(obj)
            figure, plot(obj.previewTimeHorizon, obj.ZMPConstraintXMax,'b.');
            hold on,plot(obj.previewTimeHorizon, obj.ZMPConstraintXMin,'r.');
            hold on,plot(obj.previewTimeHorizon, obj.previewZMPXPosition,'k.');
            xlabel("t(sec)");ylabel("x(m)")
            legend("ZMP x constraint max", "ZMP x constraint min", "optimal ZMP x")

            figure,plot(obj.previewTimeHorizon, obj.ZMPConstraintYMax,'b.');
            hold on,plot(obj.previewTimeHorizon, obj.ZMPConstraintYMin,'r.');
            hold on,plot(obj.previewTimeHorizon, obj.previewZMPYPosition,'k.');
            xlabel("t(sec)");ylabel("y(m)")
            legend("ZMP y constraint max", "ZMP y constraint min", "optimal ZMP y")

            figure,plot(obj.previewZMPXPosition, obj.previewZMPYPosition);
            hold on,plot(obj.previewZMPXPosition, obj.previewZMPYPosition,'.');
            axis equal
            xlabel("x(m)");ylabel("y(m)")
            legend("optimal ZMP")
            
        end

        function zmpControl = getOptimalZMP(obj)
            zmpControl = [obj.previewZMPXPosition(2); obj.previewZMPYPosition(2)];
        end


    end


end