 #! /bin/bash
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
 PATH=$PATH:/usr/bin
 JRE_HOME=/usr/bin/java
 ROS_HOSTNAME=IMU2
 DATABASE=/home/jg/db/ai
 ROSCORE_LOG=roscore.log
java -Xmx4096M --enable-native-access=ALL-UNNAMED -Djava.library.path=/usr/lib/jni --enable-preview --add-modules jdk.incubator.vector -Dtablespace=$DATABASE -cp /usr/share/jars/Ros.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/RosCore.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/RosMsgsGeom.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/Relatrix.jar:/usr/share/jars/RockSack.jar:/usr/share/jars/rocksdbjni-9.10.0-linux-aarch64.jar:/usr/share/jars/json.jar:/usr/share/jars/roscar.jar:/usr/share/jars/rknn4j.jar org.ros.RosCore $1 __master:=$ROS_HOSTNAME __ip:=$ROS_HOSTNAME>$ROSCORE_LOG&
 
