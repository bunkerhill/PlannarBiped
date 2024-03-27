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
        Up_ZMPConstraintXMin
        Up_ZMPConstraintXMax
        Up_ZMPConstraintYMin
        Up_ZMPConstraintYMax
        Low_ZMPConstraintXMin
        Low_ZMPConstraintXMax
        Low_ZMPConstraintYMin
        Low_ZMPConstraintYMax

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

            b = [obj.Up_ZMPConstraintXMax-p*p_z(1);
                 -(obj.Up_ZMPConstraintXMin-p*p_z(1));
                 obj.Low_ZMPConstraintXMax-p*p_z(1);
                 -(obj.Low_ZMPConstraintXMin-p*p_z(1))];

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
            beq1 = xi(2)-p_z(2);

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

            b = [obj.Up_ZMPConstraintYMax-p*p_z(2);
                 -(obj.Up_ZMPConstraintYMin-p*p_z(2));
                 obj.Low_ZMPConstraintYMax-p*p_z(2);
                 -(obj.Low_ZMPConstraintYMin-p*p_z(2))];
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Yz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---Y DIRECTION SOLUTION NOT FOUND---");
            end

           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   preview zmp position  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.ZMPXVelocity = Xz_dot;
            obj.ZMPYVelocity = Yz_dot;

            obj.previewZMPXPosition = zeros(2*obj.vectorLength,1);
            obj.previewZMPYPosition = zeros(2*obj.vectorLength,1);
            obj.previewZMPXPosition(1)=p_z(1);
            obj.previewZMPYPosition(1)=p_z(2);
            obj.previewZMPXPosition(obj.vectorLength+1)=p_z(1);
            obj.previewZMPYPosition(obj.vectorLength+1)=p_z(2);
            for i=1:obj.vectorLength-1
                % up
                obj.previewZMPXPosition(i+1)=obj.previewZMPXPosition(i)+obj.delta*obj.ZMPXVelocity(i);
                obj.previewZMPYPosition(i+1)=obj.previewZMPYPosition(i)+obj.delta*obj.ZMPYVelocity(i);
                % low
                obj.previewZMPXPosition(i+obj.vectorLength+1)=obj.previewZMPXPosition(obj.vectorLength+i)+obj.delta*obj.ZMPXVelocity(obj.vectorLength+i);
                obj.previewZMPYPosition(i+obj.vectorLength+1)=obj.previewZMPYPosition(obj.vectorLength+i)+obj.delta*obj.ZMPYVelocity(obj.vectorLength+i);
            end
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        end

        % Generate ZMP constraint
        function obj = ZMPConstraintSampledFromFootPlacement(obj)
            obj.Up_ZMPConstraintXMax = zeros(obj.vectorLength, 1);
            obj.Up_ZMPConstraintXMin = zeros(obj.vectorLength, 1);
            obj.Up_ZMPConstraintYMax = zeros(obj.vectorLength, 1);
            obj.Up_ZMPConstraintYMin = zeros(obj.vectorLength, 1);
            obj.Low_ZMPConstraintXMax = zeros(obj.vectorLength, 1);
            obj.Low_ZMPConstraintXMin = zeros(obj.vectorLength, 1);
            obj.Low_ZMPConstraintYMax = zeros(obj.vectorLength, 1);
            obj.Low_ZMPConstraintYMin = zeros(obj.vectorLength, 1);
            for i=1:obj.vectorLength
                stanceFootIndex=find(obj.previewTimeHorizon(i)>=obj.stanceConstraint.time,1,"last");
                obj.Up_ZMPConstraintXMax(i) = obj.stanceConstraint.Up_ankleX(stanceFootIndex)+obj.footHalfLength;
                obj.Up_ZMPConstraintXMin(i) = obj.stanceConstraint.Up_ankleX(stanceFootIndex)-obj.footHalfLength;
                obj.Up_ZMPConstraintYMax(i) = obj.stanceConstraint.Up_ankleY(stanceFootIndex)+obj.footHalfWidth;
                obj.Up_ZMPConstraintYMin(i) = obj.stanceConstraint.Up_ankleY(stanceFootIndex)-obj.footHalfWidth;
                obj.Low_ZMPConstraintXMax(i) = obj.stanceConstraint.Low_ankleX(stanceFootIndex)+obj.footHalfLength;
                obj.Low_ZMPConstraintXMin(i) = obj.stanceConstraint.Low_ankleX(stanceFootIndex)-obj.footHalfLength;
                obj.Low_ZMPConstraintYMax(i) = obj.stanceConstraint.Low_ankleY(stanceFootIndex)+obj.footHalfWidth;
                obj.Low_ZMPConstraintYMin(i) = obj.stanceConstraint.Low_ankleY(stanceFootIndex)-obj.footHalfWidth;
            end
        end


        function [] = drawZMPPreviewAndConstraint(obj)
            figure, plot(obj.previewTimeHorizon, obj.Up_ZMPConstraintXMax,'-');
            hold on,plot(obj.previewTimeHorizon, obj.Up_ZMPConstraintXMin,'-');
            hold on,plot(obj.previewTimeHorizon(1:end-1), obj.previewZMPXPosition(2:obj.vectorLength),'*-');
            hold on,plot(obj.previewTimeHorizon, obj.Low_ZMPConstraintXMax,'--');
            hold on,plot(obj.previewTimeHorizon, obj.Low_ZMPConstraintXMin,'--');
            hold on,plot(obj.previewTimeHorizon(1:end-1), obj.previewZMPXPosition(obj.vectorLength+2:2*obj.vectorLength),'*--');
            xlabel("t(sec)");ylabel("x(m)")
            legend("ZMP up x constraint max", "ZMP up x constraint min", "optimal up ZMP x","ZMP low x constraint max", "ZMP low x constraint min", "optimal low ZMP x")

            figure,plot(obj.previewTimeHorizon, obj.Up_ZMPConstraintYMax,'-');
            hold on,plot(obj.previewTimeHorizon, obj.Up_ZMPConstraintYMin,'-');
            hold on,plot(obj.previewTimeHorizon(1:end-1), obj.previewZMPYPosition(2:obj.vectorLength),'*-');
            hold on,plot(obj.previewTimeHorizon, obj.Low_ZMPConstraintYMax,'--');
            hold on,plot(obj.previewTimeHorizon, obj.Low_ZMPConstraintYMin,'--');
            hold on,plot(obj.previewTimeHorizon(1:end-1), obj.previewZMPYPosition(obj.vectorLength+2:2*obj.vectorLength),'*--');
            xlabel("t(sec)");ylabel("y(m)")
            legend("ZMP up y constraint max", "ZMP up y constraint min", "optimal up ZMP y", "ZMP low y constraint max", "ZMP low y constraint min", "optimal low ZMP y")

            figure,plot(obj.previewZMPXPosition(1:obj.vectorLength), obj.previewZMPYPosition(1:obj.vectorLength),'*-');
            hold on,plot(obj.previewZMPXPosition(obj.vectorLength+1:2*obj.vectorLength), obj.previewZMPYPosition(obj.vectorLength+1:2*obj.vectorLength),'*--');
            axis equal
            xlabel("x(m)");ylabel("y(m)")
            legend("optimal up ZMP","optimal low ZMP")
            
        end

        function zmpControl = getOptimalZMP(obj)
            zmpControl = [obj.previewZMPXPosition(2); obj.previewZMPYPosition(2)];
        end


    end


end