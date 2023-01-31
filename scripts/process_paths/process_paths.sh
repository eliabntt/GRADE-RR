#!/bin/bash
set -e
if [ -z "$1" ]
  then
    echo "at least the file to process needs to be given as arg"
    exit 1
fi

echo "Going to process file ${1}. This will create tmp/tmp.usda, a temporary processing usda, which will then be converted in a usd file"
echo

echo "We STRONGLY suggest that you review this script, it may end up overwrite or delete your files."
echo "Note that, except for that, you can run this script safely"
read -p "Are you sure you want to run this? " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

mkdir -p tmp
usdcat -o tmp/tmp.usda $1

if [ -z "$2" ]
  then
    name=$(dirname $1)
    base=$(basename $1 .usd)
    echo "No argument specified. The resulting file will be in ${name}/${base}_proc.usd"
    python ./change_paths.py --input tmp/tmp.usda --output_dir ${name} --output_name ${base}_proc
    usdcat -o ${name}/${base}_proc.usd ${name}/${base}_proc.usda
    rm ${name}/${base}_proc.usda
    rm tmp/tmp.usda
elif [ -z "$3" ]
  then
    name=$(dirname $1)
    base=$2
    basename=$(basename $1 .usd)
    echo "No output directory specified. The resulting file will be in  ${name}/${2}.usd"
    python ./change_paths.py --input $1 --output_name $2 --output_dir ${name}
    usdcat -o ${name}/${2}.usd ${name}/${2}.usda
    rm tmp/tmp.usda
    rm ${name}/${2}.usda
elif [ -z "$4" ]
  then
    echo "The resulting file will be in  ${3}/{2}.usd"
    mkdir -p $3
    python ./change_paths.py --input $1 --output_name $2 --output_dir $3
    usdcat ${3}/${2}.usda -o ${3}/${2}.usd
    rm tmp/tmp.usda
    rm ${3}/${2}.usda
fi
