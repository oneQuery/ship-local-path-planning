function [startPoint, endPoint] = angle_to_point(startPoint, angle, distance)
% Convert [starPoint, angle, distance] to [startPoint, endPoint]

DistanceX = distance * cos(angle) ;
DistanceY = distance * sin(angle) ;

endPointX = startPoint(:, 1) + DistanceX ;
endPointY = startPoint(:, 2) + DistanceY ;

endPoint = [endPointX, endPointY] ;

end

