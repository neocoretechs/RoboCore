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
java -Xmx1024m -Djava.library.path=/usr/lib/jni -Xshare:off -cp /usr/share/jars/bootstrapper.jar:/usr/share/jars/Relatrix.jar:/usr/share/jars/BigSack.jar:/usr/share/jars/commons-logging-1.1.2.jar:. com.neocoretechs.bootstrapper.bootstrap $ROS_HOSTNAME $ROS_HOSTNAME org.ros.RosRun $1 __ip:=$ROS_HOSTNAME __master:=$ROS_MASTER>$2&
 
