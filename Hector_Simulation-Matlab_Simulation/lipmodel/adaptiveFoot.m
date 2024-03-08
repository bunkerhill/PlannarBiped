classdef adaptiveFoot
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
    end
    
    methods
        function obj = adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth)
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
            obj.leftStepWidthMax=0.3;
            obj.leftStepWidthMin=0.1;
            obj.rightStepWidthMax=-0.1;
            obj.rightStepWidthMin=-0.3;

            obj.rightStepDcmOffsetMax = 1;
            obj.rightStepDcmOffsetMin = 0;
            obj.leftStepDcmOffsetMax = 0;
            obj.leftStepDcmOffsetMin= -1;

            obj.stepLengthMax=0.18; %m
            obj.stepLengthMin=-0.18;
            obj.longitudinalDCMOffsetMax=1;
            obj.longitudinalDCMOffsetMin=-1;
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

        function obj = findOptimalFootPlacement(obj, Nsteps, xi, currentStanceFoot, currentStanceFootPosition, currentTime)
            obj.leftoverTime=obj.stepDuration - mod(currentTime, obj.stepDuration);
            obj=obj.getStanceFootSequence(Nsteps, currentStanceFoot);
            obj.xiInitial = xi;
            obj.stanceFootInitial=currentStanceFootPosition;
            dcmOffsetX = xi(1)-currentStanceFootPosition(1);
            dcmOffsetY = xi(2)-currentStanceFootPosition(2);
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
            for i=2:Nsteps
                obj.stanceFootConstraint.time(i+1)=obj.stanceFootConstraint.time(i)+obj.stepDuration;
                obj.stanceFootConstraint.ankleX(i+1)=obj.stanceFootConstraint.ankleX(i)+obj.optimalStanceFootX(i);
                obj.stanceFootConstraint.ankleY(i+1)=obj.stanceFootConstraint.ankleY(i)+obj.optimalStanceFootY(i);
            end
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
            legend("stance foot x","\xi_u^x","stance foot ankle x")

            figure,plot(timeVector, stanceFootVector(2,:),'.');
            hold on,plot(timeVector, xiVector(2,:));
            hold on,plot(obj.stanceFootConstraint.time, obj.stanceFootConstraint.ankleY,'o');
            xlabel("t(sec)"); ylabel("y(m)");
            legend("stance foot y","\xi_u^y","stance foot ankle y")

            figure,plot(stanceFootVector(1,:),stanceFootVector(2,:),'*')
            hold on,plot(xiVector(1,:),xiVector(2,:));
            hold on,plot(obj.stanceFootConstraint.ankleX, obj.stanceFootConstraint.ankleY,'o');
            
            xlabel("x(m)");ylabel("y(m)")
            legend("stance foot", "\xi", "stance foot ankle");

            % figure,plot(obj.stanceFootConstraint.time, obj.stanceFootConstraint.ankleX,'.');
        end


        function obj = optimalLongitudinalFootPlacement(obj, Nsteps, xdcm, currentStanceFootPosition)
            import casadi.*
            % longitudinal dcm offset
            b = SX.sym('b', Nsteps);
            % longitudinal foot placement
            s = SX.sym('s', Nsteps);
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
            objectiveFunction = (b-obj.dcmXSteady)'*(b-obj.dcmXSteady);
            % equality constraints
            deltaT = obj.deltaTransformation(obj.stepDuration);
            deltaTLeftover = obj.deltaTransformation(obj.leftoverTime);
            g=deltaTLeftover*(s(1)+b(1)) - xdcm;
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
            dcmOffsetLowerBound= obj.longitudinalDCMOffsetMin*ones(size(b));
            dcmOffsetUpperBound= obj.longitudinalDCMOffsetMax*ones(size(b));
            stepLengthLowerBound=obj.stepLengthMin*ones(size(s));
            stepLengthUpperBound=obj.stepLengthMax*ones(size(s));
            args.lbx=[dcmOffsetLowerBound; stepLengthLowerBound];
            args.ubx=[dcmOffsetUpperBound; stepLengthUpperBound];
            args.lbg=zeros(size(g));
            args.ubg=zeros(size(g));
            args.p=[];
            dcmOffsetInitialGuess = ones(size(b))*obj.dcmXSteady;
            stanceFootInitialGuess = ones(size(s))*obj.stepLengthSteady;
            args.x0=[dcmOffsetInitialGuess; stanceFootInitialGuess];
            sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
                'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
            x_sol = full(sol.x);           % Get the solution
            dcmOptimal = x_sol(1:Nsteps);
            stepLengthOptimal = x_sol(Nsteps+1:2*Nsteps);
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
            for i=1:Nsteps
                if obj.stanceFootSeq(i)==0
                    % left foot
                    leftFoot=[leftFoot, stanceFootPosition+s(i)];
                else
                    % right foot
                    rightFoot=[rightFoot, stanceFootPosition+s(i)];
                end
                stanceFootPosition = stanceFootPosition+s(i);
            end

            % objective function
            leftFootSteadyState = obj.stepWidthSteady/2;
            rightFootSteadyState = -obj.stepWidthSteady/2;
            objectiveFunction = (leftFoot-leftFootSteadyState)*(leftFoot-leftFootSteadyState)'...
                +(rightFoot-rightFootSteadyState)*(rightFoot-rightFootSteadyState)';
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
                if i==Nsteps
                    % if this is the last step
                    dcmOffsetLowerBound(i)=obj.dcmYSteady;
                    dcmOffsetUpperBound(i)=obj.dcmYSteady;
                    dcmOffsetInitialGuess(i)=dcmOffsetUpperBound(i);
                    stanceFootInitialGuess(i)=stepWidthUpperBound(i);
                end
            end

            args.lbx = [ dcmOffsetLowerBound; stepWidthLowerBound];
            args.ubx = [ dcmOffsetUpperBound; stepWidthUpperBound];
            args.lbg = zeros(size(g));
            args.ubg = zeros(size(g));

            args.p   =  [];  % There are no parameters in this optimization problem
            dcmOffsetInitialGuess = ones(size(b));
            stanceFootInitialGuess = ones(size(s));
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
