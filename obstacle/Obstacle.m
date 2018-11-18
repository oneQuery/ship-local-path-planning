classdef Obstacle < handle
    % Configurations of obstalce
    
    properties
        position
        velocity
        radius
    end
    
    methods 

        
        function setNextPosition(obj, timeStepInterval)
            move = obj.velocity * timeStepInterval ;
            obj.position = obj.position + move ;
        end
    end
end

