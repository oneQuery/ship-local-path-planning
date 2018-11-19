%% Clear
clc; clear; close all ;
addpath(genpath('agent'), genpath('obstacle'), genpath('ship_models'), genpath('function')) ;

%% Map condition
mapBoundary.xMin = -2 ;
mapBoundary.xMax = 400 ;
mapBoundary.yMin = -200 ;
mapBoundary.yMax = 200 ;
figure_scale = 2 ;

%% Initial agent
agent = Agent() ;
agent.model = WMAV2016() ;
agent.position = [0 ;   % x(m)
                 0 ;    % y(m)
                 0] ;     % psi(rad)
agent.velocity = [1.5 ;   % u(m/s)
                  0 ;   % v(m/s)
                  0] ;  % r(rad/s)
% agent.radius = 5 ;    % (m)

%% Initial obstacles
obstacles.obstacle1 = Obstacle() ;
obstacles.obstacle1.position = [200 ;     % x(m)
                                0] ;   % y(m)
obstacles.obstacle1.velocity = [-1.5 ;     % x(m)
                                0] ;   % y(m)
obstacles.obstacle1.radius = 5 ;

obstacles.obstacle2 = Obstacle() ;
obstacles.obstacle2.position = [300 ;    % x(m)
                                150] ;   % y(m)
obstacles.obstacle2.velocity = [-1.2 ;   % x(m)    
                                -1.2] ; % y(m)      
obstacles.obstacle2.radius = 3 ;

obstacles.obstacle3 = Obstacle() ;
obstacles.obstacle3.position = [300 ;    % x(m)
                                -150] ;   % y(m)
obstacles.obstacle3.velocity = [-1.2 ;   % x(m)    
                                1.2] ; % y(m)     
obstacles.obstacle3.radius = 3 ;

obstacleNames = fieldnames(obstacles) ;

%% Time horizon
timeHorizon = 60;

%% Mission of agent
N_line_grid = 5 ;
targetLineX = linspace(-10, 800, N_line_grid)' ;
targetLineY = linspace(0, 0, N_line_grid)' ;
targetLine = [targetLineX, targetLineY] ;
% targetPoint = [0, 30] ;

%% Visualization setting
VisualizeVelocityObstacle = true ;
VisualizeReachableVelocities = true ;
VisulzeReachableAvoidanceVocities = true ;

%% Initial condition for simulation
time = 0 ;
global dt
dt = 1 ;
history_agent_position = agent.position ;

%% Simulation
while mapBoundary.xMin <= agent.position(1) && agent.position(1) <= mapBoundary.xMax ...
        && mapBoundary.yMin <= agent.position(2) && agent.position(2) <= mapBoundary.yMax
    
    %% Visualization
    simulationFigure = figure(1) ;
    figure_position = [-1700, 100] ;
    figure_size = [(mapBoundary.xMax - mapBoundary.xMin),...
                   (mapBoundary.yMax - mapBoundary.yMin)] * figure_scale;
    simulationFigure.Position = [figure_position, figure_size] ;
    daspect([1 1 1])
    grid on ;
    axis([mapBoundary.xMin, mapBoundary.xMax, mapBoundary.yMin, mapBoundary.yMax]) ;
    xlabel('x(m)') ;
    ylabel('y(m)') ;
    
    % Clear the axes
    cla ;
    hold on ;
    if time == 0
        disp('Press enter to start') ;
        pause() ;
    end
    
    % Draw the target point or line
    plot(targetLine(:, 1), targetLine(:, 2), 'r') ;
    %{
    plot(targetPoint(1), targetPoint(2), 'v') ;
    %}
    
    % Write clock
    time = time + dt ;
    timeString = ['Time: ', num2str(time), ' s'] ;
    text(mapBoundary.xMin, mapBoundary.yMax - 1, timeString) ;
    
    %% Proceed to next step
    % Calculate Collision Cone
    collisionConePoints = collision_cone(agent, obstacles, timeHorizon) ;
    
    %% Calculate velocity obstacle from Collision Cone
    for obstacleIndex = 1:numel(fieldnames(obstacles))
        velocityObstaclePoints{obstacleIndex} = ...
            collisionConePoints{obstacleIndex} + ...
            obstacles.(obstacleNames{obstacleIndex}).velocity(1:2)' ;
    end
    
    %% Draw velocity obstacle
    if VisualizeVelocityObstacle
        for j = 1:numel(fieldnames(obstacles))
            if isreal(velocityObstaclePoints{j})
                fill(velocityObstaclePoints{j}(:, 1), velocityObstaclePoints{j}(:, 2), 'y') ;
                alpha(0.1) ;
            end
        end
    end
    
    %% Calculate feasible accelearation
    agent.feasible_acceleration() ;
        
    %% Calculate reachable velocities of agent
    agent.reachableVelocities = (agent.velocity + dt * agent.feasibleAcceleration) * (1) ;
    
    %% Draw reachable velocites
    shiftedReachableVelocities = agent.position + agent.reachableVelocities ;
    if VisualizeReachableVelocities
        k = convhull(shiftedReachableVelocities(1, :),...
            shiftedReachableVelocities(2, :)) ;
        patch('Faces', 1:1:length(k),...
            'Vertices',...
            [shiftedReachableVelocities(1, k);...
            shiftedReachableVelocities(2, k)]',...
            'FaceColor', 'g', 'FaceAlpha', .1) ;
    end
    
    %% Calculate reachable avoidance velocities
    for m = 1:numel(fieldnames(obstacles))
        in = inpolygon(shiftedReachableVelocities(1, :), shiftedReachableVelocities(2, :),...
            velocityObstaclePoints{m}(:, 1), velocityObstaclePoints{m}(:, 2)) ;
        inSet(:, m) = in ;
    end
    in = any(inSet, 2) ; 
    
    shiftedReachableAvoidanceVelocities = shiftedReachableVelocities(:, ~in) ;
    reachableAvoidanceVelocities = shiftedReachableAvoidanceVelocities - agent.position ;
    if isempty(reachableAvoidanceVelocities)
        text(mapBoundary.xMin, (mapBoundary.yMin + mapBoundary.yMax) / 2, ...
            'Nothing reachable avoidance velocities', 'FontSize', 20, 'Color', 'r') ;
        break
    end
    
    %% Draw reachable avoidance velocities
    if VisulzeReachableAvoidanceVocities
        plot(shiftedReachableAvoidanceVelocities(1, :),...
            shiftedReachableAvoidanceVelocities(2, :), '.') ;
    end
  
    %% Draw agent and obstacle position
    draw_agent(agent) ;
    draw_obstacle(obstacles) ;
    
%     obstacle_color = ['k', 'r']  ;
%     
%     viscircles(agent.position(1:2)', agent.radius, 'Color', 'b', 'LineWidth', 1) ;
%     for n = 1:numel(fieldnames(obstacles))
%         viscircles(obstacles.(obstacleNames{n}).position(1:2)', ...
%             obstacles.(obstacleNames{n}).radius, ...
%             'Color', obstacle_color(n), 'LineWidth', 1) ;
%     end
        
    if mod(time, 1) == 0
        history_agent_position(:, end + 1) = agent.position ;
    end
    plot(history_agent_position(1, :), history_agent_position(2, :), '.b') ;
    
    %% Velocity strategy: to target point
    %{
    distanceToTarget = [] ;
    for l = 1:length(shiftedReachableAvoidanceVelocities)
        distanceToTarget(l, :) =...
            pdist([shiftedReachableAvoidanceVelocities(l, :); targetPoint]) ;
    end
    [~, ChosenVelocityIndex] = min(distanceToTarget) ;
    %}
    
    %% Velocity strategy: to target line
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Need to be modified %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [~, maxIndex] = maxk(shiftedReachableAvoidanceVelocities(1, :),...
        round(length(shiftedReachableAvoidanceVelocities) / (50) + 1)) ;
    [~, minIndexOfMaxIndex] = min(abs(shiftedReachableAvoidanceVelocities(2, maxIndex))) ;
    ChosenVelocityIndex = maxIndex(minIndexOfMaxIndex) ;
    
%     [~, idx_nu] = min(abs(shiftedReachableAvoidanceVelocities(2, :))) ;
%     ChosenVelocityIndex = idx_nu ;

%     [~, idx_nu] = max((shiftedReachableAvoidanceVelocities(1, :))) ;
%     ChosenVelocityIndex = idx_nu ;
%     
    %% Choose the agent's velocity
    agent.velocity = reachableAvoidanceVelocities(:, ChosenVelocityIndex) ;
    
    %% Draw the veloity vector of agent and obstacle
    quiver(agent.position(1), agent.position(2),...
        agent.velocity(1), agent.velocity(2), 0,...
        'Color', 'b', 'LineWidth', 1) ;
    for o = 1:numel(fieldnames(obstacles))
        quiver(obstacles.(obstacleNames{o}).position(1), obstacles.(obstacleNames{o}).position(2),...
            obstacles.(obstacleNames{o}).velocity(1), obstacles.(obstacleNames{o}).velocity(2), 0,...
            'Color', 'r', 'LineWidth', 1) ;
    end
    
    %% Calculate next position
    agent.setNextPosition() ;
    for p = 1:numel(fieldnames(obstacles))
        obstacles.(obstacleNames{p}).setNextPosition(dt) ;
    end
    
%     for q = 1:numel(fieldnames(obstacles))
%         if pdist([agent.position(1:2)'; obstacles.(obstacleNames{q}).position(1:2)']) <= ...
%                 agent.radius + obstacles.(obstacleNames{q}).radius 
%             text((mapBoundary.xMin + mapBoundary.xMax) / 2,...
%                 (mapBoundary.yMin + mapBoundary.yMin) / 2,...
%                 'Crash occured', 'Color', 'red', 'FontSize', 20) ;
%             break
%         end
%     end

    %% Udpate the rpm the ship took
    agent.update_rpm(ChosenVelocityIndex)
end