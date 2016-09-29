#!/bin/bash

echo "--------------------------------------------------"
echo "Script to setup Futurakart Project ROS environment"
echo "- setup alias 'fkart' : cd /to/project/dir; source devel/setup.bash"
echo "- setup ROS_MASTER_URI, ROS_HOSTNAME from ros.conf file"
echo ""
echo "Usage : source setup_all.bash"
echo ""

CURRENT_PATH="`pwd`"

function create_backup() {
    local file_to_backup=$1
    if [ -f "${file_to_backup}" ] && [ ! -f "${file_to_backup}.save" ]; then
        cp ${file_to_backup} ${file_to_backup}.save
    fi
}

function search_and_add() {
    local file=$1
    local rx_grep_1=$2
    local rx_grep_2=$3
    local rx_sed=$4
    local task_name=$5
    local line=$6

    local is_found=`cat $file | grep -iE "$rx_grep_1"`
    if [ "$is_found" == "" ]; then
        echo "$line" >> $file
        echo "=> Appended $task_name to the file $file"
    else
        local is_found=`echo $is_found | grep -iE "$rx_grep_2"`
        if [ "$is_found" == "" ]; then
            # comment previous and append new one
            sed -i "$rx_sed" $file
            echo "$line" >> $file
            echo "=> Comment previous value and appended $task_name to the file $file"
        else
            echo "=> $task_name is already defined"
        fi
    fi

}


############ Setup alias 'fkart' ################################
echo ""
echo "Setup alias 'fkart'"

alias_file="${HOME}/.bash_aliases"
alias_cmd="alias fkart=\"cd ${CURRENT_PATH}; source devel/setup.bash\""

create_backup ${alias_file}

if [ -f "$alias_file" ]; then

    search_and_add \
                $alias_file \
                "^alias fkart=\"cd .+; source devel/setup.bash\"" \
                "^alias fkart=\"cd ${CURRENT_PATH}; source devel/setup.bash\"" \
                "/^alias fkart=\"cd .*; source devel\/setup.bash\"/ s/^/#/" \
                "'fkart' alias" \
                ${alias_cmd}
else
    echo "$alias_cmd" > $alias_file
    echo "=> Created new file ~/.bash_aliases with 'fkart' alias"
fi

############ Setup ROS_MASTER_URI, ROS_HOSTNAME ################################

bashrc_file="${HOME}/.bashrc"

exec 5< ros.conf
read ros_master_uri <&5
read ros_hostname <&5

echo ""
echo "Apply ros.conf : $ros_master_uri $ros_hostname"

create_backup ${bashrc_file}

search_and_add \
                $bashrc_file \
                "^export ROS_MASTER_URI=.+" \
                "^export ${ros_master_uri}" \
                "/^export ROS_MASTER_URI=.*/ s/^/#/" \
                "ROS_MASTER_URI" \
                "export ${ros_master_uri}"

search_and_add \
                $bashrc_file \
                "^export ROS_HOSTNAME=.+" \
                "^export ${ros_hostname}" \
                "/^export ROS_HOSTNAME=.*/ s/^/#/" \
                "ROS_HOSTNAME" \
                "export ${ros_hostname}"


echo ""
echo "Execute source ~/.bashrc"
source ${HOME}/.bashrc
echo ""
echo "=> ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "=> ROS_HOSTNAME=${ROS_HOSTNAME}"
