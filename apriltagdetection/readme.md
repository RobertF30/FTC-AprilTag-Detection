# AprilTag Detection
Here is a sample code that can be used to detect the id of an AprilTag and also 
the robots position and angle relative to this tag using a standard camera (no LimeLight).
To use this you just have to download and paste the **apriltagdetection** folder in your project.
This code sample was made by Robert from team 20043 SNGine.

## Where to Start?

### Detecting only the id
If you only want to find only the id of an AprilTag there are only a few steps:

* Go to **CameraDetector** class and change the name of the camera 
to match the one in your RobotController configuration
* Go to **ExampleTeleop** and copy the line where a CameraDetector is 
instantiated (the last parameters are the tag ids - you might want to change this to match your tags)
* Call the function below in a loop to scan for an AprilTag
```
detector.scan();
```
* The function below returns the id of the last tag seen
```
detector.getLastFoundId();
```

### Detection and getting the position
If you want to also find the position of the robot relative to the AprilTag you detected there are more steps:

* Go to **AprilTagDetectionPipeline** and change the size of the tag to matches yours. (the size is in meters)
* Go to **CameraDetector** class and change the name of the camera
  to match the one in your RobotController configuration
* Also change the camera resolution
* Find your camera intrinsics and modify them in **CameraIntrinsics** class
###### or
Extend this class and put them there
###### or
Pass them directly as parameters to the AprilTagDetectionPipeline constructor
```
AprilTagDetectionPipeline(fx,fy,cx,cy);
```
This last step is a little bit tricky since there are no easy ways to get 
those values (or at least I couldn't find any).
You might want to watch a tutorial about that and/or use some software. 

* Copy the line from **CameraDetector** just like you need to do for the id (and don't forget the scan() function).

* To get the position of the robot just call:
```
detector.getPose3D(DistanceUnit.CM, AngleUnit.DEGREES);
```
or whatever measuring units you like.

If your intrinsics are not perfect don't worry there is a solution.
* Change the **distanceMultiplier** from **CameraDetector** class.
At this step you want to place your robot in front of an AprilTag. Measure the distance from the robot
to the tag. Then take the distance calculated by getPose3D(...) and compute
**measured distance** divided by **pose distance** and put the result in the **distanceMultiplier** variable.

This should be all you 