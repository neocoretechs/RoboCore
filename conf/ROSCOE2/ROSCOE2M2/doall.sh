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
 core=roscore
 coreclass=RosCore
 ROS_MASTER=ROSCOE2M2
 progs=(videorecog dostat)
 classes=(VideoObjectRecog StatusAlertSubs)
 PATH=$PATH:/usr/bin/java
 JRE_HOME=/usr/bin/java
 list() {
       if [[ "$HOSTNAME" == "$ROS_MASTER" ]] ; then echo "I Am The Master!" ; else echo "$ROS_MASTER is the Master" ; fi
	indices=(${!progs[*]})
	for ((i=0; i<${#indices[*]}; i++));
	do
    		echo shell: ${progs[${indices[i]}]}.sh
		echo class: ${classes[${indices[i]}]}
		echo
	done
}
 start() {
	# Start the master
	/home/odroid/$core.sh
 	sleep 5
 	if ps ax | grep -v "grep" | grep -q "$coreclass" ; then echo "startup"; else echo "startup FAIL"; fi
       echo
       # Attach 
	indices=(${!progs[*]})
	for ((i=0; i<${#indices[*]}; i++));
	do
    		echo ${progs[${indices[i]}]}
		prog=${progs[${indices[i]}]}
		class=${classes[${indices[i]}]}
		echo $prog $class
		/home/odroid/$prog.sh
 		sleep 2
 		if ps ax | grep -v "grep" | grep -q "$class" ; then echo "startup"; else echo "startup FAIL"; fi
		echo $prog $class
	done
 }
 stop() {
       # Stop services.
	indices=(${!progs[*]})
	for ((i=0; i<${#indices[*]}; i++));
	do
    		echo ${progs[${indices[i]}]}
		prog=${progs[${indices[i]}]}
		class=${classes[${indices[i]}]}
		echo $prog $class
 		/bin/kill -9 $(ps ax | grep '[j]ava' | grep "$class" | awk '{print $1}') 
 		sleep 5
 		if ps ax | grep -v "grep" | grep -q "$class" ; then echo "$prog shutdown FAIL"; else echo "$prog shutdown"; fi
       		echo
	done
	# shutdown master
 	/bin/kill -9 $(ps ax | grep '[j]ava' | grep "$coreclass" | awk '{print $1}') 
 	sleep 5
 	if ps ax | grep -v "grep" | grep -q "$coreclass" ; then echo "$core shutdown FAIL"; else echo "$core shutdown"; fi
       	echo
 }

 # See how we were called.
 case "$1" in
 list)
	 list
	 ;;
 start)
 	start
       ;;
 stop)
 	stop
       ;;
 restart|reload)
 	stop
 	start
 	;;
 *)
       echo $"Usage: $0 {list:start|stop|restart}"
       exit 1
 esac
 
 exit 0
