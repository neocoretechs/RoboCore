 #!/bin/bash
 #
 ### BEGIN INIT INFO
 # Provides: 
 # Should-Start:
 # Short-Description: Bring up/down 
 # Description: Bring up/down
 ### END INIT INFO
 #./rosrun.sh com.neocoretechs.robocore.VideoPubStereo videopub /home/odroid/videopubstereo.log
 # Source function library.
 . /lib/init/vars.sh
 . /lib/lsb/init-functions
 
 CWD=$(pwd)
 prog="videopubstereo"
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 ROS_HOSTNAME=IMU2
 ROS_MASTER=ROSCOE2M2
 NODE=com.neocoretechs.robocore.video.VideoPubStereo
 #LOG=/dev/null
LOG=/home/jg/video.log
java -Xmx1024M -Djava.library.path=/usr/lib/jni --enable-native-access=ALL-UNNAMED -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/json.jar org.ros.RosRun $NODE __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __name:=pubs_stereovideo>$LOG&
 
