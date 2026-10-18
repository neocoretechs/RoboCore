 #!/bin/bash
 #
 ### BEGIN INIT INFO
 # Provides: 
 # Should-Start:
 # Short-Description: Bring up/down 
 # Description: Bring up/down
 ### END INIT INFO
 # Source function library.
 . /lib/init/vars.sh
 . /lib/lsb/init-functions
 
 CWD=$(pwd)
 prog="rosrun"
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 ROS_HOSTNAME=IMU2
 ROS_MASTER=ROSCOE2M2
 ROS_NODECLASS=com.neocoretechs.robocore.propulsion.MotionController
 ROS_NODENAME=motioncontrol
 LOGFILE=/home/jg/motion.log
java -Xmx1024M --enable-preview --enable-native-access=ALL-UNNAMED --add-modules jdk.incubator.vector -Djava.library.path=/usr/lib/jni -DRoboCore.properties=/usr/share/jars/RoboCore.properties -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/RosMsgsGeom.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/jSerialComm-2.11.2.jar:/usr/share/jars/Relatrix.jar:/usr/share/jars/RockSack.jars:/usr/share/jars/rocksdbjni-9.10.0-linux-aarch64.jar:/usr/share/jars/jsoup-1.20.1.jar:/usr/share/jars/json.jar org.ros.RosRun $ROS_NODECLASS __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __name:=$ROS_NODENAME __robot:=ROSCOE2 __debug:=demuxer,marlinspike,config>$LOGFILE&
