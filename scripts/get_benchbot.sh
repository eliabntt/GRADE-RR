#/bin/bash

if [ "$#" -eq 1 ]; then
    cd $1
fi

if [ "$#" -gt 1 ]; then
    echo "illegal number of params"
    exit
fi

wget https://cloudstor.aarnet.edu.au/plus/s/n9PDshcQZiCc1h0/download -O challenge.zip
mkdir bb_challenge
unzip challenge.zip -d bb_challenge
cd bb_challenge
rm *.yaml
mv .sim_data/* ./
cd ..
#rm challenge.zip
wget https://cloudstor.aarnet.edu.au/plus/s/7lEK6dBl0zVvA5D/download -O develop.zip
mkdir bb_develop
unzip develop.zip -d bb_develop
cd bb_develop
rm *.yaml
mv .sim_data/* ./
cd ..
