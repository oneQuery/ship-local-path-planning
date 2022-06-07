# Demo
![ezgif com-gif-maker](https://user-images.githubusercontent.com/24229051/172056748-b35d3602-8a0a-4975-b665-816456b3db9a.gif)

More videos here! >> [[Videos](https://www.youtube.com/watch?v=UFRGuUviqhE&list=PLYNMvZ8JbO9YnUwum6Ky8nyXgP2XNNy-a)]

# Project Description
Ship local path planning (a.k.a. collision avoidance) using the [Velocity Obstacle](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.56.6352&rep=rep1&type=pdf) with ship safety zone. 
This was my Master's thesis topic. See the paper if you want. The link here >> [[Library](https://inha.primo.exlibrisgroup.com/discovery/fulldisplay?docid=alma991009103061005086&context=L&vid=82KST_INH:INHA&lang=ko&search_scope=Dissertation&adaptor=Local%20Search%20Engine&tab=LibraryCatalog&query=any,contains,%EA%B9%80%ED%9D%AC%EC%88%98&offset=0)]

# How to Run
Run the demo m-file:
```
>> velocityObstacleDemo
```

# Seudo Code
```
INITIALIZE agent position, agent velocity, ship safety domain, obstacles position, obstacle velocity
WHILE (the agent within the map) {
  collision cone = COMPUTE(agent velocity, agent radius, obstacle velocity, obstacle raidus)
  feasible accelerations = COMPUTE(ship dynamics, thruster specification)
  reachable velocities = feasiable accelerations * time step
  reachable avoidance velocities = reachable velocities - AREA(collision cone)
  the best velocity = PICK UP ONE THROUGH THE STRATEGY(reachable avoidance velocities)
  agent position <- agent position + (the best velocity * time step)
  obstacle position <- obstacle position + (obstacle velocity * time step)
  t <- t + time step
  }
```
