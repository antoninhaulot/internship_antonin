A project by Antonin Haulot, french intern at the University of Birmingham during the 2017 summer



​This project needs the following topics to be live in order to works : people_tracker/positions and amcl_pose


HOW TO RUN IT :

In order to launch the default code, please run the file get_people_pose.py

If you want to choose your own parameters (min and max angle for the camera, max distance between human and robot, duration of the following behavior, and speed of the camera), you can rosrun the file action_server.py and then rosrun actionlib axclient.py /follow_people

Note that the robot can sometimes be a bit slow while following somebody.
