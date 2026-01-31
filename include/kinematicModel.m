%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint index ;

            % The function returns:
            % bJi

            self.gm.updateDirectGeometry(self.gm.q); % Updates forward geometry with current config.

            Jb_i_A = zeros(3, self.gm.jointNumber); % Jacobian from b to i Angular
            Jb_i_L = zeros(3, self.gm.jointNumber); % Jacobian from b to i Linear

            
            T_i = self.gm.getTransformWrtBase(i);
            p_i = T_i(1:3,4);
   
     
          %  p_i = self.gm.iTj(1:3,4,i); % To extract the position of link i frame wrt base frame
            for j = 1:i 
                if j == 1
                    T_prev = eye(4);
                else
                    T_prev = self.gm.getTransformWrtBase(j);

                end
                    p_prev = T_prev(1:3,4); % (needed to compute position vector)
                    % (needed to compute linear velocity direction in revolut joints)
                    switch self.gm.jointAxis(j)
                        case 1  % x axes
                            z_prev = T_prev(1:3,1);  % first column of R
                        case 2  % y
                            z_prev = T_prev(1:3,2);  
                        case 3  % z
                            z_prev = T_prev(1:3,3);  
                    end

                if self.gm.jointType(j) == 0 % Revolut joint
                        Jb_i_A(:, j) = z_prev; % Direction of rotation axis
                        Jb_i_L(:, j) = cross(z_prev, (p_i - p_prev)); % Direction of linear velocity given by rotation
                end
                if self.gm.jointType(j) == 1 % Prismatic joint 
                        Jb_i_A(:, j) = zeros(3,1); % No rotation
                        Jb_i_L(:, j) = z_prev; % Linear velocity direction
                end
            end
            
            bJi = [Jb_i_A; Jb_i_L]; % Creation of complete Jacobian w.r.t. base

        end

        function bJt = getJacobianOfToolWrtBase(self)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJt of Tool wrt base.
            % The function returns:
            % bJt

            self.gm.updateDirectGeometry(self.gm.q);


                try
                    T_bt = self.gm.getToolTransformWrtBase();   % tool wrt base
                catch
                    % If tool does not exist, fallback to last link
                    T_bt = self.gm.getTransformWrtBase(self.gm.jointNumber);
                end
            
                % Position of the point where Jacobian is evaluated
                p_t = T_bt(1:3,4);
   
     
            Jb_A = zeros(3, self.gm.jointNumber); % Jacobian from b to i Angular
            Jb_L = zeros(3, self.gm.jointNumber); % Jacobian from b to i Linear

          %  p_i = self.gm.iTj(1:3,4,i); % To extract the position of link i frame wrt base frame
            for j = 1:self.gm.jointNumber 
                if j == 1
                    T_prev = eye(4);
                else
                    T_prev = self.gm.getTransformWrtBase(j);

                end
                    p_prev = T_prev(1:3,4); % (needed to compute position vector)
                    z_prev = T_prev(1:3,3);% (needed to compute linear velocity direction in revolut joints)

                if self.gm.jointType(j) == 0 % Revolut joint
                        Jb_A(:, j) = z_prev; % Direction of rotation axis
                        Jb_L(:, j) = cross(z_prev, (p_t - p_prev)); % Direction of linear velocity given by rotation
                end
                if self.gm.jointType(j) == 1 % Prismatic joint 
                        Jb_A(:, j) = zeros(3,1); % No rotation
                        Jb_L(:, j) = z_prev; % Linear velocity direction
                end
            end
            
            bJt = [Jb_A; Jb_L]; % Creation of complete Jacobian w.r.t. base

        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            try
                % If the tool exists, uses Tool Jacobian function
                self.gm.getToolTransformWrtBase(); % if tool doesn't exists, this generates an error
                self.J = self.getJacobianOfToolWrtBase();
        
            catch
                % calls the previous function and gets the jacobian of last joint (end effector)
                self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
            end
            
       end
    end
end





