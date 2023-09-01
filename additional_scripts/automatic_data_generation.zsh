#!/bin/zsh
# this sh file can be used as blueprint to generate data automatically
# please note that it is custom to our use-case
# first you need to launch the roscore using the sim time true
# then you can spawn, using screen, the rosbag record, any catkin package, and the main simulation code
# the code works with a main sleep counter, that is used to check if the simulation is still running
# if the time exceed this sleep, the system is reset automatically. Otherwise, the loop continues once the simulation is finished.

set -e
echo "Output folder: $1"
echo "Environment folder: $2"
echo "Tmp out folder: $3"
echo "Lower folder number: $4"
echo "Offset: $5"
echo "isaac folder: $6"
source ~/.zshrc
folder=${2%}/
availenv=($folder*)
folderlen=${#folder}
availenv=("${availenv[@]:$4:$5}")
# launch rostrue --- rostrue is an alias that launches roscore and set the sim time to true
screen -d -m -S ROSMASTER zsh -i -c "rostrue"

echo "WARNING! TMP OUT FOLDER WILL BE CLEARED FOR EACH EXPERIMENT"
echo "WARNING! we run rm ${3%/}/environment/* every time!!!"
echo "WARNING! we run mv ${3%/}/environment/ ${1%/}/ every time!!! Be aware!\n"

while true; do
    read -q "yn?Do you still wish to run this program (y/n)?"
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

sleep 5
mkdir -p ${1}
for env in "${availenv[@]}"
do
	currentenv=${env:$folderlen}
	echo "Processing ${currentenv}"
	rm ${3%/}/${currentenv}/ -rf
	rm ${1%/}/${currentenv}/ -rf
	mkdir -p ${1%/}/${currentenv}
	mkdir -p ${3%/}/${currentenv}
	screen -L -Logfile ${3%/}/${currentenv}/isaaclog.log -d -m -S ISAACSIM zsh -i -c "cd ${6} && ./python.sh isaac_sim_manager/simulator/paper_simulation.py --/renderer/enabled='rtx,iray'  --config='isaac_sim_manager/simulator/config.yaml' --fix_env=${currentenv} && sh isaac_sim_manager/kill.sh"
	screen -L -Logfile ${3%/}/${currentenv}/rosbag.log -d -m -S ROSRECORDER zsh -i -c "rosbag record -a -O ${1%/}/${currentenv}/${currentenv}.bag --split --size=1024 -b 0"
	screen -L -Logfile ${3%/}/${currentenv}/rviz.log -d -m -S RVIZ zsh -i -c "cd catkin_ws && source devel/setup.zsh && roslaunch exploration_manager rviz.launch"
	echo "Launched"
	sleep 200
	result=1
	cnt=90000
	while [ $result -eq 1 ]
	do
		sleep 1
		screen -wipe > suppress_output
		if ! screen -list | grep -q "ISAACSIM"; then
			rm suppress_output
			result=0
			break
		fi
		((cnt=cnt-1))
		if [[ "$cnt" -eq 0 ]]; then
			for session in $(screen -ls); do screen -S "${session}" -X quit; done
			echo "ERROR had to manually stop the sessions" > ${3}/${currentenv}/error.txt
			sleep 3
			echo "ERROR"
			screen -wipe > suppress_output
		fi
	done
	screen -d -m -S ISAACSIMkill zsh -i -c "cd ${6} && ./kill.sh"
	echo "KILLED"
	sleep 3
	#screen -S ROSRECORDER -X stuff "^C"
	screen -d -m -S ISAACRM zsh -i -c "cd ${6} && rm kit/logs -rf"
	for session in $(screen -ls | grep -o '[0-9]*\.ROSRECORDER'); do screen -S "${session}" -X stuff "^C"; done
	sleep 5
	screen -d -m -S mover${currentenv} zsh -i -c "mv ${3}/${currentenv}/* ${1}/${currentenv}/"
	echo "Finished Processing ${currentenv}"
done
