classdef ContinuousDynamics
    %CONTINUOUSDYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % @type vector of RigidBody
        RigidBodies
        
        % @type vector of GeneralizedCoordinates
        GeneralizedCoordinates
        
        % @type vector of GeneralizedVelocities
        GeneralizedVelocities

        % @type vector of GeneralizedForce
        GeneralizedForces

        % @type vector of Contact2D
        ContactConstraints
    end
    
    methods
        function obj = ContinuousDynamics(GeneralizedCoordinates, GeneralizedVelocities)
            obj.RigidBodies = [];
            obj.GeneralizedCoordinates = GeneralizedCoordinates;
            obj.GeneralizedVelocities = GeneralizedVelocities;
        end
        
        function obj = AddRigidBody(obj, RigidBody)
            obj.RigidBodies = [obj.RigidBodies, RigidBody];
        end

        function obj = AddGeneralizedForce(obj, GeneralizedForce)
            obj.GeneralizedForces = [obj.GeneralizedForces, GeneralizedForce];
        end

        function GeneralizedForceVector = getGeneralizedForceVector(obj)
            GeneralizedForceVector = zeros(length(obj.GeneralizedCoordinates), 1);
            for i=1:length(obj.GeneralizedForces)
                GeneralizedForceVector = GeneralizedForceVector + obj.GeneralizedForces(i).getGeneralizedForce(obj.GeneralizedCoordinates);
            end
        end

        function obj = AddContactConstraints(obj, ContactConstraint)
            obj.ContactConstraints = [obj.ContactConstraints, ContactConstraint];
        end

        function ContactJacobian = getContactJacobian(obj)
            % J(q) * \dot q = 0
             ContactJacobian = [];
             for i=1:length(obj.ContactConstraints)
                 ContactJacobian = [ContactJacobian; obj.ContactConstraints(i).getContactJacobian(obj.GeneralizedCoordinates)];
             end
        end

        function Jdot = getJdot(obj)
            Jdot = [];
            for i=1:length(obj.ContactConstraints)
                 Jdot = [Jdot ; obj.ContactConstraints(i).getJdot(obj.GeneralizedCoordinates, obj.GeneralizedVelocities)];
            end
        end

        function ConstraintForce = getConstraintForce(obj)
            temp0 = simplify(obj.getContactJacobian()*inv(obj.getMassMatrix()));
            temp1 = simplify(obj.getGeneralizedForceVector - obj.getCoriolisVector - obj.getGravityVector);
            temp2 = simplify(temp0*temp1);
            temp3 = simplify(temp2 + obj.getJdot()*obj.GeneralizedVelocities.');
            temp4 = simplify(temp0*(obj.getContactJacobian().'));
            ConstraintForce = simplify(-inv(temp4)*temp3);
        end

        function B_matrix = getInputMatrix(obj, inputColumnVector)
            generalizedForceVector = obj.getGeneralizedForceVector();
            B_matrix = jacobian(generalizedForceVector, inputColumnVector);
        end

        function kineticEnergy = getTotalKineticEnergy(obj)
            kineticEnergy = 0;
            for i = 1:length(obj.RigidBodies)
                kineticEnergy = kineticEnergy + obj.RigidBodies(i).getKineticEnergy(obj.GeneralizedCoordinates, obj.GeneralizedVelocities);
            end
        end

        function potentialEnergy = getTotalPotentialEnergy(obj)
            potentialEnergy = 0;
            for i = 1:length(obj.RigidBodies)
                potentialEnergy = potentialEnergy + obj.RigidBodies(i).getPotentialEnerge();
            end
        end

        function Mass = getMassMatrix(obj)
            KE = simplify(obj.getTotalKineticEnergy());
            D_mtx = simplify(jacobian(KE, obj.GeneralizedVelocities).');
            Mass = simplify(jacobian(D_mtx, obj.GeneralizedVelocities));
        end

        function Gravity = getGravityVector(obj)
            PE = simplify(obj.getTotalPotentialEnergy());
            Gravity = jacobian(PE, obj.GeneralizedCoordinates).';
            Gravity = simplify(Gravity);
        end

        function CoriolisVector = getCoriolisVector(obj)
            syms C_mtx real
            n=max(size(obj.GeneralizedCoordinates));
            Mass = obj.getMassMatrix();
            for k=1:n
	            for j=1:n
		            C_mtx(k,j)=0;
		            for i=1:n
			            C_mtx(k,j)=C_mtx(k,j)+1/2*(diff(Mass(k,j), obj.GeneralizedCoordinates(i)) + ...
				            diff(Mass(k,i), obj.GeneralizedCoordinates(j)) - ...
				            diff(Mass(i,j), obj.GeneralizedCoordinates(k)))* obj.GeneralizedVelocities(i);
		            end
	            end
            end
            C_mtx=simplify(C_mtx);

            CoriolisVector = C_mtx * obj.GeneralizedVelocities.';
        end
    end
end
