function draw_obstacle(obstacles)
%DRAW_OBSTACLE Summary of this function goes here
%   Detailed explanation goes here

obstacleNames = fieldnames(obstacles) ;

for n = 1:numel(fieldnames(obstacles))
        viscircles(obstacles.(obstacleNames{n}).position(1:2)', ...
            obstacles.(obstacleNames{n}).radius, ...
            'Color', 'r', 'LineWidth', 1) ;
        
end

