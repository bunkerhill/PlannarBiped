classdef contiMPC

    properties
        % one step time
        deta
        
        % horizon time
        T_h
        
        % foot length
        foot_length
        
        % one step length
        step_size

        % one step width
        step_width
        
        % one step time
        gait_time
        single_support_time
        double_support_time
        
        % get from main function
        % g
        g
        
        % com height
        L

        % contingency parameters
        j_max
        j_min
        a_max
        a_min
        T_u
        T_l
    end

    methods
        function obj = contiMPC(g_in,L_in,ddxy_s_max_in,ddxy_s_min_in)
            obj.g = g_in;
            obj.L = L_in;
            obj.deta = 0.01;
            obj.T_h = 1;
            obj.foot_length = 0.02;
            obj.step_width = 0.1;
            obj.step_size = 0.05;
            obj.single_support_time = 0.2;
            obj.double_support_time = 0.1;
            obj.gait_time = obj.single_support_time + obj.double_support_time;

            % contingency parameters
            obj.j_max = [1,1];
            obj.j_min = [-1,-1];
            obj.a_max = ddxy_s_max_in;
            obj.a_min = ddxy_s_min_in;
            
        end
        
        % solve the mpc problem
        function pz_dot = MPC(obj,x,p_z,ddxy_s,current_T)
            
            vector_length = round(obj.T_h/obj.deta);
            
            obj.T_u(1) = (obj.a_max(1)-ddxy_s(1))/obj.j_max(1);
            obj.T_u(2) = (obj.a_max(2)-ddxy_s(2))/obj.j_max(2);
            obj.T_l(1) = (obj.a_min(1)-ddxy_s(1))/obj.j_min(1);
            obj.T_l(2) = (obj.a_min(2)-ddxy_s(2))/obj.j_min(2);

            % objective function
            H = eye(3*vector_length);

            % equality constraint
            omega = (obj.g/obj.L)^0.5;
            lamda = exp(-omega*obj.deta);
            b_T = zeros(1,vector_length);
            for i = 1:vector_length
                b_T(i) = lamda^(i-1);
            end

            Aeq1 = (1-lamda)/omega/(1-lamda^vector_length)*b_T;
            beq1 = x(1)+x(2)/omega-p_z(1);

            Aeq2 = zeros(2,3*vector_length);
            Aeq2(1,1) = 1;
            Aeq2(1,1+vector_length) = -1;
            Aeq2(2,1) = 1;
            Aeq2(2,1+2*vector_length) = -1;


            Aeq = [blkdiag(Aeq1,Aeq1,Aeq1);
                   Aeq2];
               
            beq = [beq1 + 1/(omega^2)*ddxy_s(1)*(exp(-omega*obj.T_h)-1);
                   beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_u(1))) + obj.a_max(1)*(exp(-omega*obj.T_u(1)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_max(1)*(1-(1+obj.T_u(1)*omega)*exp(-omega*obj.T_u(1)));
                   beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_l(1))) + obj.a_min(1)*(exp(-omega*obj.T_l(1)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_min(1)*(1-(1+obj.T_l(1)*omega)*exp(-omega*obj.T_l(1)));
                   0;
                   0];

            % control constraint
            lb = [];
            ub = [];
            % inequality constraint
            p = ones(vector_length,1);
            temp = ones(vector_length);
            P = tril(temp)*obj.deta;
            A1 = [P;
                  -P];

            [X_min,X_max] = ZMP_rangex(obj,current_T);

            b1 = [X_max-p*p_z(1);
                 -(X_min-p*p_z(1))];

            A = blkdiag(A1,A1,A1);
            b = [b1;
                 b1;
                 b1];
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Xz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---X DIRECTION SOLUTION NOT FOUND---");
            end


            %----------------------------------------------------------------------------

             % objective function
            H = eye(3*vector_length);

            % equality constraint
            omega = (obj.g/obj.L)^0.5;
            lamda = exp(-omega*obj.deta);
            b_T = zeros(1,vector_length);
            for i = 1:vector_length
                b_T(i) = lamda^(i-1);
            end

            Aeq1 = (1-lamda)/omega/(1-lamda^vector_length)*b_T;
            beq1 =  x(3)+x(4)/omega-p_z(2);

            Aeq2 = zeros(2,3*vector_length);
            Aeq2(1,1) = 1;
            Aeq2(1,1+vector_length) = -1;
            Aeq2(2,1) = 1;
            Aeq2(2,1+2*vector_length) = -1;


            Aeq = [blkdiag(Aeq1,Aeq1,Aeq1);
                   Aeq2];
            beq = [beq1 + 1/(omega^2)*ddxy_s(2)*(exp(-omega*obj.T_h)-1);
                   beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_u(2))) + obj.a_max(2)*(exp(-omega*obj.T_u(2)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_max(2)*(1-(1+obj.T_u(2)*omega)*exp(-omega*obj.T_u(2)));
                   beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_l(2))) + obj.a_min(2)*(exp(-omega*obj.T_l(2)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_min(2)*(1-(1+obj.T_l(2)*omega)*exp(-omega*obj.T_l(2)));
                   0;
                   0];
            % control constraint
            lb = [];
            ub = [];
            % inequality constraint
            p = ones(vector_length,1);
            temp = ones(vector_length);
            P = tril(temp)*obj.deta;
            A1 = [P;
                  -P];

            [Y_min,Y_max] = ZMP_rangey(obj,current_T);

            b1 = [Y_max-p*p_z(2);
                 -(Y_min-p*p_z(2))];

            A = blkdiag(A1,A1,A1);
            b = [b1;
                 b1;
                 b1];
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Yz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---X DIRECTION SOLUTION NOT FOUND---");
            end

            pz_dot = [Xz_dot(1);
                      Yz_dot(1)];

        end

        function Pz_dot = MPC_horizon(obj,x,p_z,ddxy_s,current_T)
%             obj.T_h = 1;
            vector_length = round(obj.T_h/obj.deta);
            
            obj.T_u(1) = (obj.a_max(1)-ddxy_s(1))/obj.j_max(1);
            obj.T_u(2) = (obj.a_max(2)-ddxy_s(2))/obj.j_max(2);
            obj.T_l(1) = (obj.a_min(1)-ddxy_s(1))/obj.j_min(1);
            obj.T_l(2) = (obj.a_min(2)-ddxy_s(2))/obj.j_min(2);

            % objective function
            H = eye(3*vector_length);

            % equality constraint
            omega = (obj.g/obj.L)^0.5;
            lamda = exp(-omega*obj.deta);
            b_T = zeros(1,vector_length);
            for i = 1:vector_length
                b_T(i) = lamda^(i-1);
            end

            Aeq1 = (1-lamda)/omega/(1-lamda^vector_length)*b_T;
            beq1 = x(1)+x(2)/omega-p_z(1);

            Aeq2 = zeros(2,3*vector_length);
            Aeq2(1,1) = 1;
            Aeq2(1,1+vector_length) = -1;
            Aeq2(2,1) = 1;
            Aeq2(2,1+2*vector_length) = -1;


            Aeq = [blkdiag(Aeq1,Aeq1,Aeq1);
                   Aeq2];
               
            beq = [beq1 + 1/(omega^2)*ddxy_s(1)*(exp(-omega*obj.T_h)-1);
                   beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_u(1))) + obj.a_max(1)*(exp(-omega*obj.T_u(1)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_max(1)*(1-(1+obj.T_u(1)*omega)*exp(-omega*obj.T_u(1)));
                   beq1 - 1/(omega^2)*( ddxy_s(1)*(1-exp(-omega*obj.T_l(1))) + obj.a_min(1)*(exp(-omega*obj.T_l(1)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_min(1)*(1-(1+obj.T_l(1)*omega)*exp(-omega*obj.T_l(1)));
                   0;
                   0];

            % control constraint
            lb = [];
            ub = [];
            % inequality constraint
            p = ones(vector_length,1);
            temp = ones(vector_length);
            P = tril(temp)*obj.deta;
            A1 = [P;
                  -P];

            [X_min,X_max] = ZMP_rangex(obj,current_T);

            b1 = [X_max-p*p_z(1);
                 -(X_min-p*p_z(1))];

            A = blkdiag(A1,A1,A1);
            b = [b1;
                 b1;
                 b1];
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Xz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---X DIRECTION SOLUTION NOT FOUND---");
            end


            %----------------------------------------------------------------------------

             % objective function
            H = eye(3*vector_length);

            % equality constraint
            omega = (obj.g/obj.L)^0.5;
            lamda = exp(-omega*obj.deta);
            b_T = zeros(1,vector_length);
            for i = 1:vector_length
                b_T(i) = lamda^(i-1);
            end

            Aeq1 = (1-lamda)/omega/(1-lamda^vector_length)*b_T;
            beq1 =  x(3)+x(4)/omega-p_z(2);

            Aeq2 = zeros(2,3*vector_length);
            Aeq2(1,1) = 1;
            Aeq2(1,1+vector_length) = -1;
            Aeq2(2,1) = 1;
            Aeq2(2,1+2*vector_length) = -1;


            Aeq = [blkdiag(Aeq1,Aeq1,Aeq1);
                   Aeq2];
            beq = [beq1 + 1/(omega^2)*ddxy_s(2)*(exp(-omega*obj.T_h)-1);
                   beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_u(2))) + obj.a_max(2)*(exp(-omega*obj.T_u(2)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_max(2)*(1-(1+obj.T_u(2)*omega)*exp(-omega*obj.T_u(2)));
                   beq1 - 1/(omega^2)*( ddxy_s(2)*(1-exp(-omega*obj.T_l(2))) + obj.a_min(2)*(exp(-omega*obj.T_l(2)))-exp(-omega*obj.T_h) ) - 1/(omega^3)*obj.j_min(2)*(1-(1+obj.T_l(2)*omega)*exp(-omega*obj.T_l(2)));
                   0;
                   0];
            % control constraint
            lb = [];
            ub = [];
            % inequality constraint
            p = ones(vector_length,1);
            temp = ones(vector_length);
            P = tril(temp)*obj.deta;
            A1 = [P;
                  -P];

            [Y_min,Y_max] = ZMP_rangey(obj,current_T);

            b1 = [Y_max-p*p_z(2);
                 -(Y_min-p*p_z(2))];

            A = blkdiag(A1,A1,A1);
            b = [b1;
                 b1;
                 b1];
            
            options = optimset('Algorithm','interior-point-convex','Display','off');

            [Yz_dot,~,exitflag,~] = quadprog(H,[],A,b,Aeq,beq,lb,ub,[],options);
            
            if exitflag == -2
               fprintf("---X DIRECTION SOLUTION NOT FOUND---");
            end

            Pz_dot = [Xz_dot,Yz_dot];
%             obj.T_h = 1;
        end

        % Generate ZMP range
        function [X_min_next,X_max_next] = ZMP_rangex(obj,current_T)

            vector_length = round(obj.T_h/obj.deta);
            X_min_next = zeros(vector_length,1);
            X_max_next = zeros(vector_length,1);
            for i = 1:vector_length
                [X_min_next(i),X_max_next(i)] = one_ZMP_rangex(obj,current_T + (i-1)*obj.deta);
            end

        end

        function [x_min_next,x_max_next] = one_ZMP_rangex(obj,current_T)
            % start in double support
            timer = 0.2;
            if current_T < timer
                x_min_next = - 0.5 * obj.foot_length;
                x_max_next = 0.5 * obj.foot_length;

            else
                current_T = current_T - timer;
                one_gait_num = round(obj.gait_time/obj.deta);
                count = round(current_T/obj.deta);
                temp1 = floor(count/one_gait_num);
                temp2 = mod(count,one_gait_num);
                if temp2 < round(obj.single_support_time/obj.deta)
                    x_min_next = temp1 * obj.step_size - 0.5 * obj.foot_length;
                    x_max_next = temp1 * obj.step_size + 0.5 * obj.foot_length;
                else
                    x_min_next = temp1 * obj.step_size - 0.5 * obj.foot_length;
                    x_max_next = (temp1+1) * obj.step_size + 0.5 * obj.foot_length;
                end
            end
        end

        function [Y_min_next,Y_max_next] = ZMP_rangey(obj,current_T)

            vector_length = round(obj.T_h/obj.deta);
            Y_min_next = zeros(vector_length,1);
            Y_max_next = zeros(vector_length,1);
            for i = 1:vector_length
                [Y_min_next(i),Y_max_next(i)] = one_ZMP_rangey(obj,current_T + (i-1)*obj.deta);
            end

        end

        function [y_min_next,y_max_next] = one_ZMP_rangey(obj,current_T)
             % start in double support
            timer = 0.2;
            if current_T < timer
                y_min_next = -0.5 * obj.step_width - 0.5 * obj.foot_length;
                y_max_next = 0.5 * obj.step_width + 0.5 * obj.foot_length;
            else
                current_T = current_T - timer;
                one_gait_num = round(obj.gait_time/obj.deta);
                count = round(current_T/obj.deta);
                temp1 = floor(count/one_gait_num);
                temp2 = mod(count,one_gait_num);
                if temp2 < round(obj.single_support_time/obj.deta)
                    y_min_next = (-1)^temp1 * 0.5 * obj.step_width - 0.5 * obj.foot_length;
                    y_max_next = (-1)^temp1 * 0.5 * obj.step_width + 0.5 * obj.foot_length;
                else
                    y_min_next = -0.5 * obj.step_width - 0.5 * obj.foot_length;
                    y_max_next = 0.5 * obj.step_width + 0.5 * obj.foot_length;
                end
            end
        end

        function [X_min_next,X_max_next] = ZMP_rangex_plot(obj,time_period)

            vector_length = round(time_period/obj.deta);
            X_min_next = zeros(vector_length,1);
            X_max_next = zeros(vector_length,1);
            for i = 1:vector_length

                % start in double support
                current_T = (i-1)*obj.deta;
                timer = 0.2;
                if current_T < timer
                    X_min_next(i) = - 0.5 * obj.foot_length;
                    X_max_next(i) = 0.5 * obj.foot_length;
    
                else
                    current_T = current_T - timer;
                    one_gait_num = round(obj.gait_time/obj.deta);
                    count = round(current_T/obj.deta);
                    temp1 = floor(count/one_gait_num);
                    X_min_next(i) = temp1 * obj.step_size - 0.5 * obj.foot_length;
                    X_max_next(i) = temp1 * obj.step_size + 0.5 * obj.foot_length;

                end
            end

        end

        function [Y_min_next,Y_max_next] = ZMP_rangey_plot(obj,time_period)

            vector_length = round(time_period/obj.deta);
            Y_min_next = zeros(vector_length,1);
            Y_max_next = zeros(vector_length,1);
            for i = 1:vector_length

                 % start in double support
                current_T = (i-1)*obj.deta;
                timer = 0.2;
                if current_T < timer
                    Y_min_next(i) = -0.5 * obj.step_width - 0.5 * obj.foot_length;
                    Y_max_next(i) = 0.5 * obj.step_width + 0.5 * obj.foot_length;
                else
                    current_T = current_T - timer;
                    one_gait_num = round(obj.gait_time/obj.deta);
                    count = round(current_T/obj.deta);
                    temp1 = floor(count/one_gait_num);
                    Y_min_next(i) = (-1)^temp1 * 0.5 * obj.step_width - 0.5 * obj.foot_length;
                    Y_max_next(i) = (-1)^temp1 * 0.5 * obj.step_width + 0.5 * obj.foot_length;
                end

            end

        end

        function [X_min_next,X_max_next] = get_ZMP_rangex(obj,time_period)

            vector_length = round(time_period/obj.deta);
            X_min_next = zeros(vector_length,1);
            X_max_next = zeros(vector_length,1);
            for i = 1:vector_length

                % start in double support
                current_T = (i-1)*obj.deta;
                timer = 0.2;
                if current_T < timer
                    X_min_next(i) = - 0.5 * obj.foot_length;
                    X_max_next(i) = 0.5 * obj.foot_length;
    
                else
                    current_T = current_T - timer;
                    one_gait_num = round(obj.gait_time/obj.deta);
                    count = round(current_T/obj.deta);
                    temp1 = floor(count/one_gait_num);
                    temp2 = mod(count,one_gait_num);
                    if temp2 < round(obj.single_support_time/obj.deta)
                        X_min_next(i) = temp1 * obj.step_size - 0.5 * obj.foot_length;
                        X_max_next(i) = temp1 * obj.step_size + 0.5 * obj.foot_length;
                    else
                        X_min_next(i) = temp1 * obj.step_size - 0.5 * obj.foot_length;
                        X_max_next(i) = (temp1+1) * obj.step_size + 0.5 * obj.foot_length;
                    end

                end
            end

        end

        function [Y_min_next,Y_max_next] = get_ZMP_rangey(obj,time_period)

            vector_length = round(time_period/obj.deta);
            Y_min_next = zeros(vector_length,1);
            Y_max_next = zeros(vector_length,1);
            for i = 1:vector_length

                 % start in double support
                current_T = (i-1)*obj.deta;
                timer = 0.2;
                if current_T < timer
                    Y_min_next(i) = -0.5 * obj.step_width - 0.5 * obj.foot_length;
                    Y_max_next(i) = 0.5 * obj.step_width + 0.5 * obj.foot_length;
                else
                    current_T = current_T - timer;
                    one_gait_num = round(obj.gait_time/obj.deta);
                    count = round(current_T/obj.deta);
                    temp1 = floor(count/one_gait_num);
                    temp2 = mod(count,one_gait_num);
                    if temp2 < round(obj.single_support_time/obj.deta)
                        Y_min_next(i) = (-1)^temp1 * 0.5 * obj.step_width - 0.5 * obj.foot_length;
                        Y_max_next(i) = (-1)^temp1 * 0.5 * obj.step_width + 0.5 * obj.foot_length;
                    else
                        Y_min_next(i) = -0.5 * obj.step_width - 0.5 * obj.foot_length;
                        Y_max_next(i) = 0.5 * obj.step_width + 0.5 * obj.foot_length;
                    end
                end

            end

        end

    end


end