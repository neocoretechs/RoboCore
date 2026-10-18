 #!/bin/bash
 #
 ### BEGIN INIT INFO
 # Provides: 
 # Should-Start:
 # Short-Description: Bring up/down 
 # Description: Bring up/down
 ### END INIT INFO
 #./rosrun.sh com.neocoretechs.robocore.VideoPubStereo /home/odroid/videopubstereo.log
 # Source function library.
 . /lib/init/vars.sh
 . /lib/lsb/init-functions
 
 CWD=$(pwd)
 prog="videoviewer"
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 ROS_HOSTNAME=IMU
 ROS_MASTER=IMU
 NODE=com.neocoretechs.robocore.video.VideoListenerStereo
 LOG=/home/odroid/videolistenerstereo.log
java -Xmx1024M -Djava.library.path=/usr/lib/jni -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/json.jar org.ros.RosRun $NODE __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __name:=subs_stereovideo 
 
