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
 ROS_HOSTNAME=IMU
 ROS_MASTER=IMU
 ROS_NODECLASS=$1
 ROS_NODENAME=$2
 LOGFILE=$3
[[ ! -d "/usr/share/jars/$ROS_NODECLASS" ]] && mkdir "/usr/share/jars/$ROS_NODECLASS"
java -Xmx1024M -Djava.library.path=/usr/lib/jni -Djars.provision=/usr/share/jars/$ROS_NODECLASS -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RXTXcomm.jar org.ros.RosLauncher $ROS_NODECLASS __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __name:=$ROS_NODENAME __serveNode:=Control1>$LOGFILE&
