# Milestone_3
## introduction
- copied code of milestone _2.
- learned about extraction of red waypoint .
- learned about bgr value of waypoint(23,23,255)
- surfed for vision sensor and path following bot[youtube](https://www.youtube.com/watch?v=ZMI_kpNUgJM&t=1299s)
- read documentation for PID
- used [Youtube](https://www.youtube.com/watch?v=ZMI_kpNUgJM&t=1299s) for getting more idea about PID controller and its working 
- read sample code to make for crn_bot
## working
- used code of milestone_2.
- lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255]) for selecting waypoint.(image)
- calculated coordinates of waypoints for reference and bot simulation by contours 
- arranged coordinates in order nearer to far .
- taking refernce from [youtube](https://www.youtube.com/watch?v=ZMI_kpNUgJM&t=1299s) tried making  PID controller.
- defined integral , diffrential, and proportional term.
- defined function for calulating bot position w.r.t waypoints 
- found relative position of my bot w.r.t every point 
- assigned number to waypoints(image)
- PID didn't workout.
- SO I added a improvised method that will reduce it's parameters(speed or angle) and then set angle as they were all perpendicular to each other