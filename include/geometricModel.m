%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        jointAxis
        iTj
        q
        eTt  % transformation end-effector tool. Added to be coherent with main call of geometricModel
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,jointAxis,eTt)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.jointAxis = jointAxis;
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt; % added inizialization 
            else 
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %% updateDirectGeometry function
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            self.q = q;

            for t = 1:self.jointNumber % iteration for all the joints

                T_current = eye(4);
                % Revolut joint
                if (self.jointType(t) == 0)
                    switch self.jointAxis(t)
                        case 1  % rotation about x
                            R = [1,  0,                0;
                                 0,  cos(self.q(t)), -sin(self.q(t));
                                 0,  sin(self.q(t)),  cos(self.q(t))];
                        
                        case 2  % rotation about y
                            R = [ cos(self.q(t)),  0,  sin(self.q(t));
                                  0,                1,  0;
                                 -sin(self.q(t)),  0,  cos(self.q(t))];
                        
                        case 3  % rotation about z
                            R = [cos(self.q(t)), -sin(self.q(t)),  0;
                                 sin(self.q(t)),  cos(self.q(t)),  0;
                                 0,               0,               1];
                        otherwise
                            error(['Invalid axes ' num2str(t)]);
                    end
                    T_current(1:3,1:3) = R;
                end
    
                % Prismatic joint
                if (self.jointType(t) == 1)
                    switch self.jointAxis(t)
                        case 1
                            T_current(1,4) = self.q(t);  % translation along x
                        case 2
                            T_current(2,4) = self.q(t);  % translation along y
                        case 3
                            T_current(3,4) = self.q(t);  % translation along z
                        otherwise
                            error(['Invalid axes ' num2str(t)]);
                    end 
                end

                % Invalid joint
                if (self.jointType(t) ~= 0 &&  self.jointType(t) ~= 1)
                    error("Invalid joint");
                end
            
                % Computation of iTj
                self.iTj(:,:,t) = self.iTj_0(:,:,t) * T_current;
            end
        end

        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % outputs
            % bTt : transformation matrix from the manipulator base to the
            % tool
            
            bTe = self.getTransformWrtBase(self.jointNumber);
            bTt = bTe * self.eTt; % tool with respect to base

        end

        function [bTk] = getTransformWrtBase(self,k)
            %% getTransformWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            self.updateDirectGeometry(self.q); % updates the current matrix using current q vector
            bTk = eye(4); %initialization of output matrix

            for i = 1:k
                bTk = bTk * self.iTj(:,:,i); % multiplies all the matricies to obtain the transformation w.r.t base
            end
        end
    end
end


