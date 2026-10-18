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
 prog="vidplayback"
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 ROS_HOSTNAME=ROSCOE1
 ROS_MASTER=ROSCOE1
 NODE=com.neocoretechs.robocore.video.VideoPlaybackStereo
 LOG=/home/odroid/videoplaystereo.log
java -Xmx1024M -Djava.library.path=/usr/lib/jni -cp /usr/share/jars/Relatrix.jar:/usr/share/jars/BigSack.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RXTXcomm.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/v4l4j.jar $NODE $ROS_MASTER $ROS_HOSTNAME 9500 
 
