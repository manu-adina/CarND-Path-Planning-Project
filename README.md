# CarND-Path-Planning-Project
Due to the simplicity of the task, the complex logic behind having a state system and cost functions can be avoided for this project to complete the objectives. However, including those functions is definitely required for having a good self driving vehicle that can operate in complex road conditions.  

But for this project, a simple idea was implemented using the logic from the blog provided in the chatroom (2nd link below). 
  
Spline Generation and Creating Waypoints - Source: <https://youtu.be/7sI3VHFPP0w>    
Prediction and Behaviour - Source: <https://medium.com/intro-to-artificial-intelligence/path-planning-project-udacitys-self-driving-car-nanodegree-be1f531cc4f7>
   
### Prediction
1. First check in what lane the neighbour car is. Using the information extracted from the sensor fusion, the car can be determined on what lane it is. Since the road is 4m wide, this means that from the left, the first 4m (0 - 4m) are going to be taken by the left lane, the next 4m (4 - 8m) by the middle lane and the last 4m (8 - 12m) by the right lane.

	```
	if(d > 0 && d < 4) {
		neighbour_car_lane = 0; // Left lane = 0
	} else if(d > 4 && d < 8) {
		neighbour_car_lane = 1; // Middle lane = 1
	} else if(d > 8 and d < 12) {
		neighbour_car_lane = 2; // Right lane = 2
	}
	```

2. Then the neighbour car's s is predicted to make further decisions later on.

	```
	neighbour_car_s += ((double)prev_size*0.02*check_speed);
	```
3. Based on that, then we create a flag depending on where the neighbour car is in relation to the ego car. The neighbour car has to be within 30m of the ego car to be flagged.

	```
	if(neighbour_car_lane == ego_car_lane) {
		neighbour_car_ahead |= neighbour_car_s > car_s && (neighbour_car_s - car_s) < 30;
	} else if((neighbour_car_lane - ego_car_lane) == -1) {
		neighbour_car_left |= (car_s + 30) > neighbour_car_s  && (car_s - 30) < neighbour_car_s;
	} else if((neighbour_car_lane - ego_car_lane) == 1) {
		neighbour_car_right |= (car_s + 30) > neighbour_car_s  && (car_s - 30) < neighbour_car_s;
	}
	```

### Behaviour
1. Then the flags are used to decide on what action is to perform.
	    
	Do:  
	**Left lane switch** when no neighbour cars are in the left lane, and the ego car is not in the left lane.  
	**Right lane switch** when no neighbour cars are in the right lane, and the ego car is not in the right lane.  
	**Slow down** if can't perform either operations.  
	**Speed up** if there are no cars ahead

	```
	if(neighbour_car_ahead) {
		if(!neighbour_car_left && ego_car_lane > 0) {
			ego_car_lane--;
		} else if(!neighbour_car_right && ego_car_lane !=2) {
			ego_car_lane++;
		} else {
			ref_vel -= speed_step;
		}
	} else if(ref_vel < max_speed){
		ref_vel += speed_step;
	}
	```
2. Once the value `ego_car_lane` is changed, the rest of the code which was provided by Udacity's tutorial handles the spline formation to perform the lane change.  

	Three waypoints are generated for the spline formation. Notice that the waypoints are generated based on `ego_car_lane` value.

	```
	vector<double> next_wp_0 = getXY(car_s + 30, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp_1 = getXY(car_s + 60, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp_2 = getXY(car_s + 90, (2+4*ego_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	```
3. Using the three waypoints, the spline is generated using the `spline.h` library.

	```
	tk::spline s;
	s.set_points(ptsx, ptsy);
	```
4. This spline `s` object is then used to give the corresponding 'y' value based on given 'x' value. These x and y values are then used as a path for the car to drive.