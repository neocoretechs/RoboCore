 #!/bin/sh
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
 prog="ps3"
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 ROS_MASTER=odroid64
 ROS_HOSTNAME=$(ip addr show wlan2 | grep 'state UP' -A2 | grep 'inet\b' | tail -n1 | awk '{print $2}' | cut -f1 -d'/')
while [ -z "$ROS_HOSTNAME" ]
do
 sleep 1
 ROS_HOSTNAME=$(ip addr show wlan2 | grep 'state UP' -A2 | grep 'inet\b' | tail -n1 | awk '{print $2}' | cut -f1 -d'/')
done
PS3CONTROLLER='Logitech, Inc F310 Gamepad [XInput Mode]'
#PS3CONTROLLER='Logitech Logitech Dual Action'
#PS3CONTROLLER='USB Joystick'
PS3CONTROLLERTYPE=gamepad
#PS3CONTROLLERTYPE=joystick
java -Xmx256M -Djava.library.path=/usr/lib/jni -Djinput.plugins=net.java.games.input.LinuxEnvironmentPlugin -cp /usr/share/java/jinput.jar:/usr/share/relatrix/relatrix.jar:/usr/share/relatrix/bigsack.jar:/usr/share/relatrix/commons-lang-2.6.jar:/usr/share/relatrix/ws-commons-util-1.0.1.jar:/usr/share/relatrix/commons-digester-1.8.1.jar:/usr/share/relatrix/commons-logging-1.1.2.jar:/usr/share/relatrix/commons-io-2.4.jar:/usr/share/relatrix/RosBase.jar:/usr/share/relatrix/commons-net-1.4.1.jar:/usr/share/relatrix/RosMsgs.jar:/usr/share/relatrix/RosMsgsGeom.jar:/usr/share/relatrix/Ros.jar:/usr/share/relatrix/commons-codec-1.9.jar:/usr/share/relatrix/RoboCore.jar:/usr/share/relatrix/VoxHumana.jar:/usr/share/relatrix/cmu_time_awb.jar:/usr/share/relatrix/cmu_us_kal.jar:/usr/share/relatrix/cmudict04.jar:/usr/share/relatrix/cmulex.jar:/usr/share/relatrix/cmutimelex.jar:/usr/share/relatrix/en_us.jar:/usr/share/relatrix/freetts.jar:/usr/share/relatrix/freetts-jsapi10.jar:/usr/share/relatrix/jsapi.jar:/usr/share/relatrix/mbrola.jar:/usr/share/relatrix/jinput.jar org.ros.RosRun com.neocoretechs.robocore.PS3CombinedPubs __master:=$ROS_MASTER __ip:=$ROS_HOSTNAME __motorcontrol:=$ROS_HOSTNAME __controller:=$PS3CONTROLLER __type:=$PS3CONTROLLERTYPE __debug:=true>ps3.log&
 
