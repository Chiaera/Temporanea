function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Check that R is a valid rotation matrix using IsRotationMatrix().

    if (IsRotationMatrix(R) == true)

        theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
     
        % Check if the configuration is unique
        if cos(theta) == 0
            error("The inverse mapping is not unique")
        else
            phi = atan2(R(3,2), R(3,3));
            psi =  atan2(R(2,1), R(1,1));
            disp('Theta, phi and psi are unique');
            
        end
    end
end

