function collisionConeBeyondTimeHorizonPoints = collision_cone(agent, obstacles, timeHorizon)
% Calculate Collision Cone sector as a convex hull
% AHat: mapped circle of agent
% BHat: mapped circle of obstacle
% tangent1, tangent2 : tangets from AHat to BHat
% tangent1, tangent2 representation: [starting point, angle]
% collision cone representation: [tangent1; tangent2]

obstacleNames = fieldnames(obstacles) ;

for obstacleIndex = 1:numel(obstacleNames)
    obstacle = obstacles.(obstacleNames{obstacleIndex}) ;
    
    %% Map into Configuration Space
    % About penetrating line through the centers of A and B
    losLength = pdist([agent.position(1:2)'; obstacle.position(1:2)']) ;
    
    losDifference = obstacle.position(1:2) - agent.position(1:2) ;
    losAngleEarth = atan2(losDifference(2), losDifference(1)) ;
    
    losAngleBody = [losAngleEarth - atan2(agent.velocity(2), agent.velocity(1)) ;
                    pi - (losAngleEarth - atan2(agent.velocity(2), agent.velocity(1))) ;
                    -pi/2 + (losAngleEarth - atan2(agent.velocity(2), agent.velocity(1))) ;
                    pi/2 + (losAngleEarth - atan2(agent.velocity(2), agent.velocity(1)))] ;
                    
    

    
    % Angles for collision cone
    radii = obstacle.radius + [agent.Rf; agent.Ra; agent.Rp; agent.Rs] ;
    lengthToRadiiVertex = sqrt(losLength^2 + radii.^2 - 2*losLength.*radii.*(cos(losAngleBody))) ;  % wrong
    
%     angleToRadiiVertex = losAngleEarth + asin(radii./lengthToRadiiVertex) ;

    
%     AHatRadius = agent.radius - agent.radius ;
%     BHatRadius1 = quaternion_radii(contactAngle1);
%     BHatRadius2 = quaternion_radii(contactAngle2);
%     BHatRadius1 = ;
%     BHatRadius2 = ;
% 
    AHatCenter = agent.position(1:2)' ;
%     BHatCenter = obstacle.position(1:2)' ;
    
    %% Calculate the two tangents from AHat to BHat as a [start point, angle]
    
    
    % Angle between tangent1 and the center penetrating line
%     tempAngle1 = asin(BHatRadius/tempDistance) ;
    tempAngle1 = max(asin(radii./lengthToRadiiVertex.*sin(losAngleBody))) ;
    tempAngle2 = min(asin(radii./lengthToRadiiVertex.*sin(losAngleBody))) ;
    

    
    % Angle of tangent1 and tangent2
    tangent1Angle = losAngleEarth + tempAngle1 ;
    tangent2Angle = losAngleEarth + tempAngle2 ;
    
    %% Initial Collision Cone
    % Collision Cone Points
    [collisionConePoint1, collisionConePoint2] = angle_to_point(AHatCenter, [tangent1Angle; tangent2Angle], 1e+3) ;
    collisionConePoints =...
        [collisionConePoint1; collisionConePoint2] ;
    
    %% Collision Cone before time horizon
%     dm = pdist([agent.position(1:2)'; obstacle.position(1:2)']) - agent.radius - obstacle.radius;
%     timeHorizonVelocity = dm / timeHorizon ;
%         
%     angleStep = sort(linspace(tangent1Angle, tangent2Angle, 30))' ;
%     [collisionConeTimeHorizonPoint1, collisionConetimeHorizonPoint2] = ...
%           angle_to_point(AHatCenter, angleStep, timeHorizonVelocity) ;
%     collisionConeTimeHorizonPoints = ...
%           [collisionConeTimeHorizonPoint1; collisionConetimeHorizonPoint2] ;
%     
%     collisionConeBeyondTimeHorizonPoints{obstacleIndex} = ...
%         [collisionConePoints(2:end, :); collisionConeTimeHorizonPoints(2:end, :)] ;
      
    %% For new time horizon velocities
    CHat_scale = 1 / timeHorizon ;
    CHat_center(1) = AHatCenter(1) + losDifference(1) * CHat_scale ;
    CHat_center(2) = AHatCenter(2) + losDifference(2) * CHat_scale ;
    CHat_theta = pi / 2 - tempAngle1 ;
    CHat_abs_theta1 = (losAngleEarth + pi) + CHat_theta ; 
    CHat_abs_theta2 = (losAngleEarth + pi) - CHat_theta ;
    Chat_theta_grid = sort(linspace(CHat_abs_theta1, CHat_abs_theta2, 3), 'descend')' ;
    
    [collisionConeTimeHorizonPoint1, collisionConetimeHorizonPoint2] = ...
        angle_to_point(CHat_center, Chat_theta_grid, (agent.Rf + agent.Ra) * CHat_scale) ;
    collisionConeTimeHorizonPoints = ...
        [collisionConeTimeHorizonPoint1; collisionConetimeHorizonPoint2] ;

    collisionConeBeyondTimeHorizonPoints{obstacleIndex} = ...
        [collisionConePoints(2:end, :); collisionConeTimeHorizonPoints(2:end, :)] ;
end
end

