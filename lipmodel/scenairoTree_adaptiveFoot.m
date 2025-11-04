classdef scenairoTree_adaptiveFoot
    %ADAPTIVEFOOT Summary of this class goes here
    %   Detailed explanation goes here
    properties
        % 9.8m/s^2
        g

        % COM height
        h
        
        % sqrt(g/h)
        omega

        % time length of each step
        stepDuration
        % the time duration before this step finishes
        leftoverTime

        % average longitudinal speed
        averageSpeed

        % steady state step length
        stepLengthSteady
        % steady state step width
        stepWidthSteady

        dcmXSteady

        dcmYSteady

        % number of steps planned
        Nsteps
        % stance foot sequence, vector of size Nsteps. 0: left foot stance,
        % 1: right foot stance. Only the future planned stance foot, 
        % Do not include current stance foot
        stanceFootSeq

        % Constraints
        leftStepWidthMax
        leftStepWidthMin
        rightStepWidthMax
        rightStepWidthMin
        % lateral dcm offset constraint
        leftStepDcmOffsetMax
        leftStepDcmOffsetMin
        rightStepDcmOffsetMax
        rightStepDcmOffsetMin
        % longitudinal dcm offset constraint
        longitudinalDCMOffsetMax
        longitudinalDCMOffsetMin
        % longitudinal step length
        stepLengthMax
        stepLengthMin

        % optimal foot placement
        optimalStanceFootX;
        optimalStanceFootY;
        % optimal DCM offset
        optimalDCMOffsetX;
        optimalDCMOffsetY;
        % 
        xiInitial
        stanceFootInitial

        % stance foot constraint used by ZMP MPC
        stanceFootConstraint

        % current acc
        acc

        dcmOffsetX
        dcmOffsetY

        BNmin
        BNmax
        dNmin
        dNmax

        deltaT
        disturbance
    end
    
    methods
        function obj = scenairoTree_adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth)
            %ADAPTIVEFOOT Construct an instance of this class
            %   Detailed explanation goes here
            obj.g = 9.8; % m/s^2
            obj.h = comHeight; %m
            obj.omega = sqrt(obj.g / obj.h);
            obj.stepDuration = stepDuration; % sec
            obj.averageSpeed = averageSpeed; % m/s
            obj.stepLengthSteady = obj.averageSpeed * obj.stepDuration;
            obj.stepWidthSteady = stepWidth;

            obj.dcmXSteady = obj.stepLengthSteady/(1/obj.deltaTransformation(obj.stepDuration) -1);
            obj.dcmYSteady = obj.stepWidthSteady/(1/obj.deltaTransformation(obj.stepDuration) +1);
            % set constraints
            obj.leftStepWidthMax=0.4;
            obj.leftStepWidthMin=0.05;
            obj.rightStepWidthMax=-0.05;
            obj.rightStepWidthMin=-0.4;

            obj.rightStepDcmOffsetMax = 0.3;
            obj.rightStepDcmOffsetMin = 0;
            obj.leftStepDcmOffsetMax = 0;
            obj.leftStepDcmOffsetMin= -0.3;

            obj.stepLengthMax=0.4; %m
            obj.stepLengthMin=-0.4;
            obj.longitudinalDCMOffsetMax=0.3;
            obj.longitudinalDCMOffsetMin=-0.3;
        end
        
        function outputArg = deltaTransformation(obj,timeDuration)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = exp(-obj.omega*timeDuration);
        end

        function outputArg = inverseDeltaTrans(obj,inputArg)
            outputArg = log(inputArg)/(-obj.omega);
        end


        function obj = getStanceFootSequence(obj, Nsteps, currentStanceFoot)
            % Given current stance foot, find the next Nsteps stance
            % sequence.
            % currentStanceFoot 0: left foot is stance foot; 1: right
            % foot is stance foot.
            obj.Nsteps = Nsteps;
            obj.stanceFootSeq = zeros(1, Nsteps);
            for i=1:Nsteps    
                obj.stanceFootSeq(i)=mod(currentStanceFoot+i,2);
            end
        end

        function obj = findOptimalFootPlacement(obj, Nsteps, xi, currentStanceFoot, currentStanceFootPosition, currentTime, acc)
            obj.leftoverTime=obj.stepDuration - mod(currentTime, obj.stepDuration);
            obj=obj.getStanceFootSequence(Nsteps, currentStanceFoot);
            obj.xiInitial = xi;
            obj.stanceFootInitial=currentStanceFootPosition;
            dcmOffsetX = xi(1)-currentStanceFootPosition(1);
            dcmOffsetY = xi(2)-currentStanceFootPosition(2);
            obj.dcmOffsetX = dcmOffsetX;
            obj.acc = acc;
            obj=obj.optimalLongitudinalFootPlacement(Nsteps, dcmOffsetX, currentStanceFootPosition(1));
            obj=obj.optimalLateralFootPlacement(Nsteps, dcmOffsetY, currentStanceFootPosition(2));
            obj.stanceFootConstraint = struct;
            obj.stanceFootConstraint.time = zeros(1,Nsteps+1);
            obj.stanceFootConstraint.ankleX=zeros(1,Nsteps+1);
            obj.stanceFootConstraint.ankleY=zeros(1,Nsteps+1);
            obj.stanceFootConstraint.time(1)=currentTime;
            obj.stanceFootConstraint.ankleX(1)=currentStanceFootPosition(1);
            obj.stanceFootConstraint.ankleY(1)=currentStanceFootPosition(2);
            obj.stanceFootConstraint.ankleX(2)=obj.stanceFootConstraint.ankleX(1)+obj.optimalStanceFootX(1);
            obj.stanceFootConstraint.ankleY(2)=obj.stanceFootConstraint.ankleY(1)+obj.optimalStanceFootY(1);
            
            obj.stanceFootConstraint.time(2)=currentTime+obj.leftoverTime;
            % for i=2:Nsteps
            %     obj.stanceFootConstraint.time(i+1)=obj.stanceFootConstraint.time(i)+obj.stepDuration;
            %     obj.stanceFootConstraint.ankleX(i+1)=obj.stanceFootConstraint.ankleX(i)+obj.optimalStanceFootX(i);
            %     obj.stanceFootConstraint.ankleY(i+1)=obj.stanceFootConstraint.ankleY(i)+obj.optimalStanceFootY(i);
            % end
        end

        function [] = drawOptimalFootPlacement(obj)
            T= (obj.stepDuration-obj.leftoverTime):0.01:obj.stepDuration;
            stanceFootVector = [obj.stanceFootInitial(1)*ones(size(T)); obj.stanceFootInitial(2)*ones(size(T))];
            dcmOffsetInitial = obj.xiInitial - obj.stanceFootInitial;
            xiX=dcmOffsetInitial(1)*exp(obj.omega*(T-T(1)))+obj.stanceFootInitial(1);
            xiY=dcmOffsetInitial(2)*exp(obj.omega*(T-T(1)))+obj.stanceFootInitial(2);

            timeVector=T;
            xiVector=[xiX; xiY];
            for i=1:obj.Nsteps
                timeNextStep = timeVector(end):0.01:timeVector(end)+obj.stepDuration;
                timeVector=[timeVector, timeNextStep];
                stanceFootNextStep = [stanceFootVector(1,end)+obj.optimalStanceFootX(i); 
                                      stanceFootVector(2,end)+obj.optimalStanceFootY(i)];
                stanceFootVector=[stanceFootVector, [stanceFootNextStep(1)*ones(size(timeNextStep));
                                                     stanceFootNextStep(2)*ones(size(timeNextStep))]];
                xiX=(xiVector(1,end)-stanceFootNextStep(1))*exp(obj.omega*(timeNextStep-timeNextStep(1)))+stanceFootNextStep(1);
                xiY=(xiVector(2,end)-stanceFootNextStep(2))*exp(obj.omega*(timeNextStep-timeNextStep(1)))+stanceFootNextStep(2);
                xiVector=[xiVector, [xiX;xiY]];
            end
            figure,plot(timeVector, stanceFootVector(1,:),'.');
            hold on,plot(timeVector, xiVector(1,:));
            hold on,plot(obj.stanceFootConstraint.time, obj.stanceFootConstraint.ankleX,'o');
            xlabel("t(sec)"); ylabel("x(m)");
            legend("stance foot x","xi","stance foot ankle x")

            figure,plot(timeVector, stanceFootVector(2,:),'.');
            hold on,plot(timeVector, xiVector(2,:),'.');
            hold on,plot(obj.stanceFootConstraint.time, obj.stanceFootConstraint.ankleY,'o');
            xlabel("t(sec)"); ylabel("y(m)");
            legend("stance foot y","xi","stance foot ankle y")

            figure,plot(stanceFootVector(1,:),stanceFootVector(2,:),'*')
            hold on,plot(xiVector(1,:),xiVector(2,:));
            hold on,plot(obj.stanceFootConstraint.ankleX, obj.stanceFootConstraint.ankleY,'o');
            
            xlabel("x(m)");ylabel("y(m)")
            legend("stance foot", "xi", "stance foot");


            % figure,plot(obj.stanceFootConstraint.time, obj.stanceFootConstraint.ankleX,'.');
        end


        function obj = optimalLongitudinalFootPlacement(obj, Nsteps, xdcm, currentStanceFootPosition)
            a_min = -8;
            a_max = 8;
            j_min = -54;
            j_max = 54;
            import casadi.*
            % % longitudinal dcm offset
            % b = SX.sym('b', Nsteps);
            % % longitudinal foot placement
            % s = SX.sym('s', Nsteps);
            % ------------ decision variables ------------
            % states b at levels 1..3 (non-terminal), and b at level 4 (terminal leaves)
            % controls u at levels 1..3 (note: level-1只有一个、level-2有2个、level-3有4个)
            u1 = SX.sym('u_lev1', 1);          % control at level-1 (shared)
            
            bL2 = SX.sym('b_lev2', 2, 1);      % level-2 states (2 nodes)
            u2  = SX.sym('u_lev2', 2, 1);      % level-2 controls
            
            bL3 = SX.sym('b_lev3', 4, 1);      % level-3 states (4 nodes)
            u3  = SX.sym('u_lev3', 4, 1);      % level-3 controls
            
            bL4 = SX.sym('b_lev4', 8, 1);      % terminal states (8 leaves)

            b = [bL2;bL3;bL4];
            s = [u1;u2;u3];

            leftFoot=[];
            rightFoot=[];
            stanceFootPosition=currentStanceFootPosition;
            for i=1:Nsteps
                if obj.stanceFootSeq(i)==0
                    % left foot
                    leftFoot=[leftFoot, stanceFootPosition+s(i)];
                else
                    rightFoot=[rightFoot, stanceFootPosition+s(i)];
                end
                stanceFootPosition=stanceFootPosition+s(i);
            end
            % objective function
                % scenario probability for each node (equal split)
            prob = cell(4,1);
            prob{1} = 1;                                        % root
            prob{2} = 0.5*ones(1,2);                            % after 1 split
            prob{3} = 0.25*ones(1,4);                           % after 2 splits
            prob{4} = 0.125*ones(1,8);                          % leaves

            % objectiveFunction = (b-obj.dcmXSteady)'*(b-obj.dcmXSteady);
            w1 = 1.0; w2 = 1e-2; wN = 5.0; 
            J = 0;
            b_ss = obj.dcmXSteady;
            u_ss = obj.stepLengthSteady;
            % level-1: 两个子节点共享一个 u1，但阶段代价计在“到达 level-1 的状态上”
            % 这里按“非预见性”/场景权重，把 level-1 的两个节点都计入：
            J = J + prob{2}(1)* ( w1*(bL2(1)-b_ss)^2 + w2*(u1-u_ss)^2 );
            J = J + prob{2}(2)* ( w1*(bL2(2)-b_ss)^2 + w2*(u1-u_ss)^2 );
            
            % level-2:
            for i=1:2
                J = J + prob{3}(2*i-1)* ( w1*(bL3(2*i-1)-b_ss)^2 + w2*(u2(i)-u_ss)^2 );
                J = J + prob{3}(2*i  )* ( w1*(bL3(2*i  )-b_ss)^2 + w2*(u2(i)-u_ss)^2 );
            end
            
            % terminal (level-4 leaves):
            for i=1:8
                J = J + prob{4}(i) * wN*(bL4(i)-b_ss)^2;
            end
            % equality constraints
            deltaT = exp(obj.omega*obj.stepDuration);
            deltaTLeftover = exp(obj.omega*obj.leftoverTime);

            obj.deltaT = deltaT;

            % Calculate all d min and max
            disturbance = two_step_bounds(obj.acc(1), a_min, a_max, j_min, j_max, obj.stepDuration, obj.omega, obj.leftoverTime);
            obj.disturbance = disturbance;

            % level-1 的两个子节点状态是 bL2(1), bL2(2)：
            %   bL2(1) = a*b0 - u1 + d0_min
            %   bL2(2) = a*b0 - u1 + d0_max
            g = bL2(1) - (deltaTLeftover*xdcm - u1 + disturbance.dMin(1));
            g = [g; bL2(2) - (deltaTLeftover*xdcm - u1 + disturbance.dMax(1))];

            % --- level-2 dynamics: each bL2(i) -> bL3 children with d1 min/max ---
            d11_min = disturbance.dMin(2); d11_max = disturbance.dMax(2);
            d12_min = disturbance.dMin(3); d12_max = disturbance.dMax(3);
            a = deltaT;
            % children: bL3(1:2) are from bL2(1),  bL3(3:4) are from bL2(2)
            g = [g;
                 bL3(1) - (a*bL2(1) - u2(1) + d11_min);
                 bL3(2) - (a*bL2(1) - u2(1) + d11_max);
                 bL3(3) - (a*bL2(2) - u2(2) + d12_min);
                 bL3(4) - (a*bL2(2) - u2(2) + d12_max)];

            % --- level-3 dynamics: each bL3(i) -> bL4 children with d2 min/max ---
            d21_min = disturbance.dMin(4); d21_max = disturbance.dMax(4);
            d22_min = disturbance.dMin(5); d22_max = disturbance.dMax(5);
            d23_min = disturbance.dMin(6); d23_max = disturbance.dMax(6);
            d24_min = disturbance.dMin(7); d24_max = disturbance.dMax(7);
            % children mapping: bL4(1:2) from bL3(1); (3:4) from 2; (5:6) from 3; (7:8) from 4
            g = [g;
                 bL4(1) - (a*bL3(1) - u3(1) + d21_min);
                 bL4(2) - (a*bL3(1) - u3(1) + d21_max);
                 bL4(3) - (a*bL3(2) - u3(2) + d22_min);
                 bL4(4) - (a*bL3(2) - u3(2) + d22_max);
                 bL4(5) - (a*bL3(3) - u3(3) + d23_min);
                 bL4(6) - (a*bL3(3) - u3(3) + d23_max);
                 bL4(7) - (a*bL3(4) - u3(4) + d24_min);
                 bL4(8) - (a*bL3(4) - u3(4) + d24_max)];

            p=[];
            % Decision variables are dcmOffset b and step width s.
            nlp_prob = struct('f', J, 'x', [b;s], 'g',g,'p',p);
            opts = struct;
            opts.ipopt.max_iter = 100;
            opts.ipopt.print_level = 0; %0,3
            opts.print_time = 0; %0,1
            opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
            opts.ipopt.acceptable_obj_change_tol = 1e-6; 
            solver = nlpsol('solver', 'ipopt', nlp_prob,opts);
            args = struct;
            dcmOffsetLowerBound= obj.longitudinalDCMOffsetMin*ones(6,1);
            dcmOffsetUpperBound= obj.longitudinalDCMOffsetMax*ones(6,1);
            dNmax = d24_max*ones(8,1);
            uNmax = obj.stepLengthMax*ones(8,1);
            dNmin = d21_min*ones(8,1);
            uNmin = obj.stepLengthMin*ones(8,1);
            BNmax = (uNmax-dNmax)/(deltaT-1);
            BNmin = (uNmin-dNmin)/(deltaT-1);

            obj.BNmax = BNmax;
            obj.BNmin = BNmin;

            obj.dNmax = dNmax;
            obj.dNmin = dNmin;

            stepLengthLowerBound=obj.stepLengthMin*ones(size(s));
            stepLengthUpperBound=obj.stepLengthMax*ones(size(s));
            args.lbx=[dcmOffsetLowerBound; BNmin; stepLengthLowerBound];
            args.ubx=[dcmOffsetUpperBound; BNmax; stepLengthUpperBound];
            args.lbg=zeros(size(g));
            args.ubg=zeros(size(g));
            args.p=[];
            dcmOffsetInitialGuess = ones(size(b))*obj.dcmXSteady;
            stanceFootInitialGuess = ones(size(s))*obj.stepLengthSteady;
            args.x0=[dcmOffsetInitialGuess; stanceFootInitialGuess];
            sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
                'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
            x_sol = full(sol.x);           % Get the solution
            dcmOptimal = x_sol(1:14);
            stepLengthOptimal = x_sol(14+1:21);
            obj.optimalDCMOffsetX = dcmOptimal;
            obj.optimalStanceFootX = stepLengthOptimal;
        end

        function obj = optimalLateralFootPlacement(obj, Nsteps, dcm, currentStanceFootPosition)
            import casadi.*
            % dcm offset
            b = SX.sym('b', Nsteps);
            % lateral foot placement
            s = SX.sym('s', Nsteps);
            leftFoot = [];
            rightFoot = [];
            stanceFootPosition = currentStanceFootPosition;
            leftDCM = [];
            rightDCM = [];
            for i=1:Nsteps
                if obj.stanceFootSeq(i)==0
                    % left foot
                    leftFoot=[leftFoot, stanceFootPosition+s(i)];
                    leftDCM = [leftDCM, b(i)];
                else
                    % right foot
                    rightFoot=[rightFoot, stanceFootPosition+s(i)];
                    rightDCM = [rightDCM, b(i)];
                end
                stanceFootPosition = stanceFootPosition+s(i);
            end

            % objective function
            leftFootSteadyState = obj.stepWidthSteady/2;
            rightFootSteadyState = -obj.stepWidthSteady/2;
            objectiveFunction = 1*((leftFoot-leftFootSteadyState)*(leftFoot-leftFootSteadyState)'...
                +(rightFoot-rightFootSteadyState)*(rightFoot-rightFootSteadyState)') ...
                + 1*((leftDCM + obj.dcmYSteady)*(leftDCM + obj.dcmYSteady)'...
                + (rightDCM - obj.dcmYSteady)*(rightDCM - obj.dcmYSteady)');
            % equality constraint
            deltaT = obj.deltaTransformation(obj.stepDuration);
            deltaTLeftover = obj.deltaTransformation(obj.leftoverTime);
            g=deltaTLeftover*(s(1)+b(1)) - dcm;
            for i=2:Nsteps
                g=[g; deltaT*(s(i)+b(i))-b(i-1)];
            end
            p=[];
            % Decision variables are dcmOffset b and step width s.
            nlp_prob = struct('f', objectiveFunction, 'x', [b;s], 'g',g,'p',p);
            opts = struct;
            opts.ipopt.max_iter = 100;
            opts.ipopt.print_level = 0; %0,3
            opts.print_time = 0; %0,1
            opts.ipopt.acceptable_tol =1e-8; % optimality convergence tolerance
            opts.ipopt.acceptable_obj_change_tol = 1e-6; 
            solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

            args = struct;
            dcmOffsetLowerBound=zeros(size(b));
            dcmOffsetUpperBound=zeros(size(b));
            stepWidthLowerBound=zeros(size(s));
            stepWidthUpperBound=zeros(size(s));
            % initial guess
            dcmOffsetInitialGuess = ones(size(b));
            stanceFootInitialGuess = ones(size(s));
            for i=1:Nsteps
                if obj.stanceFootSeq(i)==0
                    % left foot
                    dcmOffsetLowerBound(i)=obj.leftStepDcmOffsetMin;
                    dcmOffsetUpperBound(i)=obj.leftStepDcmOffsetMax;
                    stepWidthLowerBound(i)=obj.leftStepWidthMin;
                    stepWidthUpperBound(i)=obj.leftStepWidthMax;
                    dcmOffsetInitialGuess(i)=dcmOffsetUpperBound(i);
                    stanceFootInitialGuess(i)=stepWidthUpperBound(i);
                else
                    % right foot
                    dcmOffsetLowerBound(i)=obj.rightStepDcmOffsetMin;
                    dcmOffsetUpperBound(i)=obj.rightStepDcmOffsetMax;
                    stepWidthLowerBound(i)=obj.rightStepWidthMin;
                    stepWidthUpperBound(i)=obj.rightStepWidthMax;
                    dcmOffsetInitialGuess(i)=dcmOffsetUpperBound(i);
                    stanceFootInitialGuess(i)=stepWidthUpperBound(i);
                end
                % if i==Nsteps
                %     % if this is the last step
                %     dcmOffsetLowerBound(i)=obj.dcmYSteady;
                %     dcmOffsetUpperBound(i)=obj.dcmYSteady;
                %     dcmOffsetInitialGuess(i)=dcmOffsetUpperBound(i);
                %     stanceFootInitialGuess(i)=stepWidthUpperBound(i);
                % end
            end

            args.lbx = [ dcmOffsetLowerBound; stepWidthLowerBound];
            args.ubx = [ dcmOffsetUpperBound; stepWidthUpperBound];
            args.lbg = zeros(size(g));
            args.ubg = zeros(size(g));

            args.p   =  [];  % There are no parameters in this optimization problem
            dcmOffsetInitialGuess = zeros(size(b));
            stanceFootInitialGuess = zeros(size(s));
            args.x0  = [dcmOffsetInitialGuess; stanceFootInitialGuess]; % initialization of the optimization problem
            
            sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
                'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
            x_sol = full(sol.x);           % Get the solution

            dcmOptimal = x_sol(1:Nsteps);
            stepWidthOptimal = x_sol(Nsteps+1:2*Nsteps);
            obj.optimalDCMOffsetY = dcmOptimal;
            obj.optimalStanceFootY = stepWidthOptimal;
        end

        

        function [] = drawPeriodicGait(obj,Nsteps)
            stanceFootXInit = 0;
            stanceFootYInit = obj.stepWidthSteady/2;
            stanceFootX=stanceFootXInit;
            stanceFootY=stanceFootYInit;
            dcmXInit=obj.dcmXSteady;
            dcmYInit=-obj.dcmYSteady;
            timestamp=0;
            T=timestamp:0.01:obj.stepDuration;
            timeVector=T;
            stanceFootXVector = stanceFootXInit*ones(size(T));
            stanceFootYVector = stanceFootYInit*ones(size(T));
            xiX=dcmXInit*(exp(obj.omega*T))+stanceFootXInit;
            xiY=dcmYInit*(exp(obj.omega*T))+stanceFootYInit;
            xiXVector = xiX;
            xiYVector = xiY;
            for i=1:Nsteps
                T=timeVector(end):0.01:timeVector(end)+obj.stepDuration;
                timeVector=[timeVector, T];
                stanceFootX=stanceFootX+obj.stepLengthSteady;
                stanceFootY=stanceFootY+ (-1)^i*obj.stepWidthSteady;
                stanceFootXVector=[stanceFootXVector, stanceFootX*ones(size(T))];
                stanceFootYVector=[stanceFootYVector, stanceFootY*ones(size(T))];
                dcmOffsetX=xiXVector(end)-stanceFootX;
                dcmOffsetY=xiYVector(end)-stanceFootY;
                xiX=dcmOffsetX*(exp(obj.omega*(T-T(1))))+stanceFootX;
                xiY=dcmOffsetY*(exp(obj.omega*(T-T(1))))+stanceFootY;
                xiXVector=[xiXVector, xiX];
                xiYVector=[xiYVector, xiY];
            end
            % x-t plot
            figure,plot(timeVector, stanceFootXVector);
            hold on,plot(timeVector, xiXVector);
            legend("stance foot x", "\xi_u^x")
            xlabel("t(sec)");ylabel("x(m)")

            % y-t plot
            figure,plot(timeVector, stanceFootYVector,'.');
            hold on,plot(timeVector, xiYVector);
            legend("stance foot y", "\xi_u^y")
            xlabel("t(sec)");ylabel("y(m)")

            % x-y plot
            figure,plot(stanceFootXVector, stanceFootYVector, '*');
            hold on,plot(xiXVector, xiYVector,'.');
            legend("stance foot", "\xi_u")
            xlabel("x(m)");ylabel("y(m)")
            axis equal
        end
    end
end

function out = two_step_bounds(a0, a_min, a_max, j_min, j_max, Tc, omega, leftoverTime)
% TWO_STEP_BOUNDS
% 输入:
%   a0     : 第一步开始时加速度  a(t_k)
%   a_min  : 加速度下界
%   a_max  : 加速度上界
%   j_min  : jerk 下界 (<0)
%   j_max  : jerk 上界 (>0)
%   Tc     : 每步时长
%   omega  : LIP 频率 ( >0 )
% 输出:
%   out.aEndMin(1:2) : 第1/2步结束(=下一步开始)的 a_min
%   out.aEndMax(1:2) : 第1/2步结束(=下一步开始)的 a_max
%   out.dMin(1:2)    : 第1/2步的 d_{k+i, min}
%   out.dMax(1:2)    : 第1/2步的 d_{k+i, max}

% ---------- 第 1 步：d_0 与下一步的起始加速度 ----------
[d0_min, d0_max] = d_bounds_from_start(a0, a_min, a_max, j_min, j_max, leftoverTime, omega);
a1_min = max(a_min, a0 + j_min*leftoverTime);
a1_max = min(a_max, a0 + j_max*leftoverTime);
% ---------- 第 2 步：d_11 和 d_12 与下一步的起始加速度 a_21 & a_22 ----------
[d11_min, d11_max] = d_bounds_from_start(a1_min, a_min, a_max, j_min, j_max, Tc, omega);
[d12_min, d12_max] = d_bounds_from_start(a1_max, a_min, a_max, j_min, j_max, Tc, omega);
% start with a1_min
a21_min = max(a_min, a1_min + j_min*Tc);
a21_max = min(a_max, a1_min + j_max*Tc);
% start with a1_max
a22_min = max(a_min, a1_max + j_min*Tc);
a22_max = min(a_max, a1_max + j_max*Tc);
% ---------- 第 3 步：d_21 d_22 d_23 和 d_24 ----------
[d21_min, d21_max] = d_bounds_from_start(a21_min, a_min, a_max, j_min, j_max, Tc, omega);
[d22_min, d22_max] = d_bounds_from_start(a21_max, a_min, a_max, j_min, j_max, Tc, omega);
[d23_min, d23_max] = d_bounds_from_start(a22_min, a_min, a_max, j_min, j_max, Tc, omega);
[d24_min, d24_max] = d_bounds_from_start(a22_max, a_min, a_max, j_min, j_max, Tc, omega);
% 汇总
out.dMin  = [d0_min; d11_min; d12_min; d21_min; d22_min; d23_min; d24_min];
out.dMax  = [d0_max; d11_max; d12_max; d21_max; d22_max; d23_max; d24_max];
end

% ====== 子函数：给定“该步起点可达加速度区间”→ 求该步的 d 区间 ======
function [d_min, d_max] = d_bounds_from_start(a_start, ...
    a_min, a_max, j_min, j_max, Tc, omega)
% 说明：
% d 对加速度单调递减 ⇒
%   d_max 由“最小加速度轨迹”取得：从 a_start_min 以 j_min 下降→撞到 a_min → 保持
%   d_min 由“最大加速度轨迹”取得：从 a_start_max 以 j_max 上升→撞到 a_max → 保持

% 命中边界所需时间（带饱和）
t_down = min(Tc, max(0, safe_div(a_min - a_start, j_min))); % j_min<0
t_up   = min(Tc, max(0, safe_div(a_max - a_start, j_max))); % j_max>0

% 积分核的两个基本量 I0/I1
I0 = @(T) exp(omega*Tc) * (1 - exp(-omega*T)) / omega;
I1 = @(T) exp(omega*Tc) * (1 - exp(-omega*T).*(1 + omega*T)) / (omega^2);

% 最小加速度轨迹: 先线性落到 a_min，再保持
int_minA = a_start*I0(t_down) + j_min*I1(t_down) ...
         + a_min*(I0(Tc) - I0(t_down));
d_max    = -(1/omega) * int_minA;

% 最大加速度轨迹: 先线性升到 a_max，再保持
int_maxA = a_start*I0(t_up) + j_max*I1(t_up) ...
         + a_max*(I0(Tc) - I0(t_up));
d_min    = -(1/omega) * int_maxA;
end

% 安全除法（避免 j=0 的数值问题）
function y = safe_div(num, den)
if abs(den) < 1e-12
    y = inf*sign(num); % 不会在 min/max 后造成问题
else
    y = num/den;
end
end


