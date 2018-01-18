#!/bin/bash

read -p "Please enter your SUNetID: " sunetid

rm -f "${sunetid}_hw1.zip"
echo "Creating ${sunetid}_hw1.zip"
zip -q "${sunetid}_hw1.zip" "submit_hw1_code_and_writeup.sh"
zip -qd "${sunetid}_hw1.zip" "submit_hw1_code_and_writeup.sh" # making an empty zip file

for fname in "P1_optimal_control.py" \
             "P2_differential_flatness.py" \
             "P3_pose_stabilization.py" \
             "P4_trajectory_tracking.py" \
             "traj_data_differential_flatness.npy" \
             "traj_data_optimal_control.npy"
do
    if [ -f $fname ]; then
        zip "${sunetid}_hw1.zip" $fname
    else
        read -p "$fname not found. Skip it? [yn]: " yn
        case $yn in
            [Yy]* ) ;;
            * ) exit;;
        esac
    fi
done

if [ -f "$sunetid.pdf" ]; then
    zip "${sunetid}_hw1.zip" "$sunetid.pdf"
else
    echo "Cannot find ./$sunetid.pdf; you must submit your HW1 writeup as $sunetid.pdf in this directory."
    exit
fi

echo ""
echo "Querying Stanford server (AFS) to determine submission number; enter SUNetID password if prompted."
lastsub=$(ssh -o NumberOfPasswordPrompts=1 -o ControlMaster=auto -o ControlPersist=yes -o ControlPath=~/.ssh/%r@%h:%p $sunetid@cardinal.stanford.edu ls -t /afs/ir/class/aa274/HW1 2>/dev/null | grep -m 1 ${sunetid}_code_and_writeup_submission_[0-9]*.zip | grep -Eo 'submission_[0-9]{1,4}' | grep -Eo '[0-9]{1,4}') # very unorthodox
ssh_success=$?
if [ $ssh_success -ne "0" ]; then
    tput setaf 1
    tput bold
    echo "
             / ██████╗ ███████╗████████╗██████╗ ██╗   ██╗██╗
  \    /\   |  ██╔══██╗██╔════╝╚══██╔══╝██╔══██╗╚██╗ ██╔╝██║
   )  ( ') <   ██████╔╝█████╗     ██║   ██████╔╝ ╚████╔╝ ██║
  (  /  )   |  ██╔══██╗██╔══╝     ██║   ██╔══██╗  ╚██╔╝  ╚═╝
   \(__)|   |  ██║  ██║███████╗   ██║   ██║  ██║   ██║   ██╗
             \ ╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝
An error occurred when connecting to the submission server. Please retry.
(One cause of this error message is entering your password incorrectly.)"
    exit
fi

subnum=$((lastsub + 1))
echo "Copying to AFS (running command below); enter SUNetID password if prompted."
set -x
scp -o NumberOfPasswordPrompts=1 -o ControlMaster=auto -o ControlPersist=yes -o ControlPath=~/.ssh/%r@%h:%p "${sunetid}_hw1.zip" "$sunetid@cardinal.stanford.edu:/afs/ir/class/aa274/HW1/${sunetid}_code_and_writeup_submission_$subnum.zip" 2>/dev/null
{ set +x; } 2>/dev/null

ssh_success=$?
if [ $ssh_success -ne "0" ]; then
    tput setaf 1
    tput bold
    echo "
             / ██████╗ ███████╗████████╗██████╗ ██╗   ██╗██╗
  \    /\   |  ██╔══██╗██╔════╝╚══██╔══╝██╔══██╗╚██╗ ██╔╝██║
   )  ( ') <   ██████╔╝█████╗     ██║   ██████╔╝ ╚████╔╝ ██║
  (  /  )   |  ██╔══██╗██╔══╝     ██║   ██╔══██╗  ╚██╔╝  ╚═╝
   \(__)|   |  ██║  ██║███████╗   ██║   ██║  ██║   ██║   ██╗
             \ ╚═╝  ╚═╝╚══════╝   ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝
An error occurred when connecting to the submission server. Please retry."
    exit
else
    tput setaf 2
    tput bold
    echo "
             / ███████╗██╗   ██╗ ██████╗ ██████╗███████╗███████╗███████╗██╗
  \    /\   |  ██╔════╝██║   ██║██╔════╝██╔════╝██╔════╝██╔════╝██╔════╝██║
   )  ( ') <   ███████╗██║   ██║██║     ██║     █████╗  ███████╗███████╗██║
  (  /  )   |  ╚════██║██║   ██║██║     ██║     ██╔══╝  ╚════██║╚════██║╚═╝
   \(__)|   |  ███████║╚██████╔╝╚██████╗╚██████╗███████╗███████║███████║██╗
             \ ╚══════╝ ╚═════╝  ╚═════╝ ╚═════╝╚══════╝╚══════╝╚══════╝╚═╝"
fi