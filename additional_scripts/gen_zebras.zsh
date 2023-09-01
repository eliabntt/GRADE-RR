#!/bin/zsh
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
	screen -d -m -S ISAACSIM zsh -i -c "cd ${6} && ./python.sh GRADE-RR/simulator/zebra_datagen.py --/renderer/enabled='rtx,iray'  --config='/media/ebonetto/WindowsData/ov/isaac_sim-2021.2.1/GRADE-RR/simulator/configs/config_zebra_datagen.yaml' --fix_env=${currentenv} && sh GRADE-RR/kill.sh"
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
	screen -d -m -S ISAACRM zsh -i -c "cd ${6} && rm kit/logs -rf"
done
