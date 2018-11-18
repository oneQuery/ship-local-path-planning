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
    AHatRadius = agent.radius - agent.radius ;
    BHatRadius = obstacle.radius + agent.radius;
    
    AHatCenter = agent.position(1:2)' ;
    BHatCenter = obstacle.position(1:2)' ;
    
    %% Calculate the two tangents from AHat to BHat as a [start point, angle]
    % Distance between the centers of AHat and BHat
    tempDistance = pdist([agent.position(1:2)'; obstacle.position(1:2)']) ;
    
    % Angle between tangent1 and the center penetrating line
    tempAngle1 = asin(BHatRadius/tempDistance) ;
    
    % Center line angle of Collision Cone
    Difference = obstacle.position(1:2) - agent.position(1:2) ;
    centerLineAngle = atan2(Difference(2), Difference(1)) ;
    
    % Angle of tangent1 and tangent2
    tangent1Angle = centerLineAngle + tempAngle1 ;
    tangent2Angle = centerLineAngle - tempAngle1 ;
    
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
    CHat_center(1) = AHatCenter(1) + Difference(1) * CHat_scale ;
    CHat_center(2) = AHatCenter(2) + Difference(2) * CHat_scale ;
    CHat_theta = pi / 2 - tempAngle1 ;
    CHat_abs_theta1 = (centerLineAngle + pi) + CHat_theta ; 
    CHat_abs_theta2 = (centerLineAngle + pi) - CHat_theta ;
    Chat_theta_grid = sort(linspace(CHat_abs_theta1, CHat_abs_theta2, 3), 'descend')' ;
    
    [collisionConeTimeHorizonPoint1, collisionConetimeHorizonPoint2] = ...
        angle_to_point(CHat_center, Chat_theta_grid, BHatRadius * CHat_scale) ;
    collisionConeTimeHorizonPoints = ...
        [collisionConeTimeHorizonPoint1; collisionConetimeHorizonPoint2] ;

    collisionConeBeyondTimeHorizonPoints{obstacleIndex} = ...
        [collisionConePoints(2:end, :); collisionConeTimeHorizonPoints(2:end, :)] ;
end
end

