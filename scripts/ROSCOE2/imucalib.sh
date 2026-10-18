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
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 LOGFILE=imu.log
 CLASS=com.neocoretechs.robocore.serialreader.IMUSerialDataPort
java -Xmx1024M --enable-preview --add-modules jdk.incubator.vector -Djava.library.path=/usr/lib/jni -DRoboCore.properties=/usr/share/jars/RoboCore.properties -cp /usr/share/jars/RosCore.jar:/usr/share/jars/RosBase.jar:/usr/share/jars/Ros.jar:/usr/share/jars/RosMsgs.jar:/usr/share/jars/RosMsgsGeom.jar:/usr/share/jars/commons-logging-1.1.2.jar:/usr/share/jars/RoboCore.jar:/usr/share/jars/jSerialComm-2.11.2.jar:/usr/share/jars/Relatrix.jar:/usr/share/jars/RockSack.jars:/usr/share/jars/rocksdbjni-9.10.0-linux-aarch64.jar $CLASS
