# RoboCup 2017 - CoSpace

This program was developed at the RoboCup 2017 competition in Nagoya, Japan.

We competed in the *RoboCupJunior CoSpace Rescue Challenge* category as team **Talentum** and reached **2nd place**.

Event: https://www.robocup2017.org/
CoSpace: http://cospacerobot.org/competition/cospace-rescue

**Team members:**

* Zoltán Hanesz
* Juraj Mičko
* Zoltán Szitás
* Ádám Urbán

There is a [GUI app](https://github.com/jjurm/robocup17-gui) we developed for generating code for the entities that are described in the strategy below.

---

## Strategy

We set our goals to devise a concept along with more modules to help us compose a tailor-made program once the map is brought out. Our approach can be divided into top, middle and bottom layer. The description is very general and implementation details including the formulas of all the computations can be found in the code.

### Top layer

The main idea is to define a *flow*, i.e. a rule for the robot to follow that is defined for all position on the map. We assign each point a motion vector, or a direction that the robot is supposed to pursue. We do this by incorporating more flow entities that are used for calculating the final motion vector for a given position. Each entity results in a vector and these vectors are then added up to form the motion vector. The flow entities are namely:

* **Flow point** – an entity that has a position, a radius and a direction. A flow point pulls the robot in a weighted average of the following directions:
    
    * (1) The direction from the robot’s position to the flow point’s position
    * (2) The direction of the flow point
    
    When the robot is far from the flow point, (2) makes an insignificant proportion of the weighted average and the robot is therefore pulled roughly in the direction to the flow point. As the robot approaches, (1) gains more weight and the robot is pulled more in the direction of the flow point. The radius of the flow point determines the scale of the distances.
    
* **Flow route** – a polygonal chain given by a list of flow points. The flow points are connected to create a continuously defined entity. Can form either a closed circuit or an open polygonal line.
* **Borders** – in order to prevent the robot from crossing borders of the map, the motion vector is affected by a vector pointing away from borders. The size (force) of this effect increases when the robot approaches the border.
* **Randomness** – the program stores a direction that is randomly altered in each step and that contributes to the motion vector. Flow point’s radius is also used to determine the size of this random vector (with a greater radius, the randomness is higher). What is noticeable, we don’t have to take a separate care of what to do when the robot deviates from the usual path. Thanks to the design, the flow will make it continue in the right direction.

Robot’s role in this model is to follow the motion vector in each step by adjusting its orientation appropriately. Having a map, we model the entities so that wherever the robot plunges, it will always follow the right and optimal route.

On top of that, we call an Environment a set of entities that define a flow. The program is capable of having more environments defined and of switching between them based on arbitrary conditions. For example, we can have several flows for collecting objects and switch between them when the robot reaches certain areas that trigger the switch; or temporarily switch to a “deposit flow” (e.g. a flow that will navigate the robot to the nearest deposit) once the robot is full until it deposits. Apart from that, we can monitor fertility of the environments in terms of “points per minute” and then let the program automatically switch between the environments to ensure the greatest fertility.

### Middle layer

During a regular round, there are numerous actions the robot has to perform, some of them long-running (following a route, navigating to a deposit area) and the others temporary (avoiding an obstacle). Having these actions, we statically assigned them a priority and a condition that determines if it’s reasonable to execute the particular action. Then we modelled a chain of these actions ordered by their priority. Whenever there is an action whose condition is true, we jump to its execution. Otherwise, the program proceeds with further actions in the chain with lower priorities and does the same – checks if their condition is true.

What happens is simple – during a run, robot usually executes actions with low priorities, such as following a route, navigating to a superobject etc. When it encounters an object to collect, an action to collect it (obviously having higher priority) happens to have its condition true. As a result, the robot will temporarily switch its execution from following a route to collecting the object. Once the object is collected, the condition of the action becomes false which in turn makes the robot continue with actions that have lower priority.

### Bottom layer

In this part of the program, we implement the basics, of which some examples include:

* **Representing robot’s position, speed and rotation** in vectors
* **Distinguishing underlying surface** – we simply look at robot’s sensors and compare the measured colours with pre-defined colour ranges to evaluate the type of surface the robot is currently on. Based on distinguishing the colours this way we are able to reliably decide what action to do, e.g. to collect underlying points. 
* **Turning to a point** – If we want the robot to change its direction to a particular coordinate, we calculate the angle difference between the robot’s directional vector and its relative vector to the destination, then adjust the rotation speed taking the remaining angle into account in order to make it more precise
* **Navigating to a point** – we persistently monitor robot’s direction relative to the target to keep him on track. We also consider the distance to the target and the pre-defined level of danger at its current location to adjust speed accordingly. When following a flow, we move forward and adjust the direction at the same time.
* **Avoiding dangerous zones and obstacles** (detected by sensors) – Thanks to sensor measurements, we can make the robot aware of the surroundings and prevent him from getting stuck in a place by defining a technique to avoid these areas should the robot encounter them.
* **Estimating robot’s coordinates in areas with position info lost** – we always remember robot’s last known coordinates and if the position info is lost, we try to monitor robot’s motion and rotational speed based on the propulsion of its motors. Based on that, we estimate the robot's position and direction. That way we can continue with navigating to a destination for some time until the precise location regained.
* **Randomness** – to prevent the robot designed to follow some patters from visiting the same areas more times, we generate randomness that is used to alternate robot’s movements.
* **Conditions of actions** – different sets of conditions to make an abstraction on telling the robot what it should do. For example, a condition whether the robot wants to go to deposit may take into account how many objects are already collected and whether a superobject is collected, prefer to collect RGB sets, prioritize depositing in the last seconds of the round, etc.
* **Area groups** – there are more groups of areas, for instance, a group of areas where the robot can increase its speed because they contain no objects to collect
* **Hunting superobjects** – We put all spawned superobjects to a pile. Having defined a set of line segments that form walls through which the robot can’t go, for a given superobject position the program can easily check if the robot is able to directly reach the superobject without crashing any walls. We do this by iterating through all pre-defined walls and checking if there is any intersection of the wall and the line going from robot to the superobject’s location. Of course, we take the robot’s width into account. If the path is free, we switch current action to navigating to the superobject by following a temporary flow that is generated with a simple flow route consisting of one line connecting the robot and the superobject.
* **Superobject rules** – on a map, there may be an area which will not be visited by the robot either because no objects spawn there or we chose a better path. In that case, the robot would not collect a superobject if it was created in such area. To solve this, we introduce a superobject rules which are defined by a target area (an area the superobject must be in for the rule to take effect), a route (a list of points that take the robot to that target area) and an entry area (an area the robot has to be in for the rule to take effect). If needed, we can define a superobject rule to allow the robot to reach superobjects even if they spawn in desolate areas. After collecting such a superobject, the flow will take care of navigating the robot back.

### Advantages of this approach

* **The vector field is continuous** – this prevents robot from not having defined its behaviour at all in particular areas
* **The whole map is covered** – we don’t have to explicitly tell the robot what to do in separate areas. It is able to follow the optimal route regardless of where it is. 
* **Handles interruptions well** – robot’s behaviour may always be interrupted, for instance by a penalty or a collision with the opponent’s robot. In these cases, the error or divergence in its path is seamlessly handled by the flow’s design.
* **State in the flow not persisted** – if we were following a route defined by a list of points, we would have to remember the position of the robot in the route in order to be able to tell what point of the route is the next. But this practice doesn’t handle interruptions well. Actually, in the previous years, we relied on following a polygonal chain and it caused significant problems. However, with the flow, the robot’s direction is ultimately given only by its position and therefore does not depend on not being unexpectedly deviated from its path.

### Further extensions

* **Pushout vectors** – unipolar magnets that repel. These entities can be used for marking dangerous zones and helping the robot avoid such areas. The pull direction is just the direction of the relative vector from the pushout vector’s position to the robot’s current position. The force of a pushout vector can depend on the distance of the robot.
* **Monitoring fertility for pre-defined areas** – we either manually or automatically divide the map into separate areas. While collecting points, we calculate PPM (= points per minute, i.e. the average speed of collecting points). The calculation always contributes to the area the robot is currently in. With these calculations, we adjust robot’s behaviour to stay in fertile areas for a longer time and remain less in those that are less fertile. Adjusting the length of stay is done by specifying the amount of randomness it uses for moving – more randomness means wandering for a longer time.
* **Dynamic randomness** - conforming how long should robot stay in a particular area by taking into account how long can he continue in doing that without having reached the maximum number of collected points, which can be estimated by considering the time elapsed from the last deposit and the number of points collected so far.
* **PID** - Use a proportional–integral–derivative controller for operations such as turning to a specific direction
* **Avoid position info lost** - detect and keep track of areas with position info lost and avoid them, e.g. by dynamically defining pushout vectors at that locations.

## GUI App

There is a [GUI app](https://github.com/jjurm/robocup17-gui) we developed for generating code for the flow entities.

![screenshot1](https://raw.githubusercontent.com/jjurm/robocup17-gui/master/screenshots/screenshot1.PNG)
![screenshot2](https://raw.githubusercontent.com/jjurm/robocup17-gui/master/screenshots/screenshot2.PNG)
