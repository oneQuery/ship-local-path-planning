function reachableVelocity = reachable_velocity(velocity, feasibleAcceleration, dt)
%REACHABLE_VELOCITY 이 함수의 요약 설명 위치
%   자세한 설명 위치

reachableVelocity = (velocity + dt * feasibleAcceleration) * 0.9 ;

end

