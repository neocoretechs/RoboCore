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
 ROS_HOSTNAME=ROSCOE1
 ROS_MASTER=ROSCOE1
 LOGFILE=imu.log
 ROS_NODENAME=pubs_imu
 ROS_NODECLASS=com.neocoretechs.robocore.navigation.IMUPubs
 IMU_PORT=/dev/ttyS1
java -Xmx1024M --enable-preview --add-modules jdk.incubator.vector -Djava.library.path=/usr/lib/jni -DRoboCore.properties=/usr/share/jars/RoboCore.properties -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/RosMsgsGeom.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/jSerialComm-2.11.2.jar:/usr/share/jars/Relatrix.jar:/usr/share/jars/RockSack.jars:/usr/share/jars/rocksdbjni-9.10.0-linux-aarch64.jar org.ros.RosRun $ROS_NODECLASS __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __name:=$ROS_NODENAME __port:=$IMU_PORT __debug:=false>$LOGFILE&
