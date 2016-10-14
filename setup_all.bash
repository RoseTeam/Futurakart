#!/bin/bash

echo "--------------------------------------------------"
echo "Script to setup Futurakart Project ROS environment"
echo "- setup alias 'fkart' : cd /to/project/dir; source devel/setup.bash"
echo "- setup ROS_MASTER_URI, ROS_HOSTNAME from ros.conf file"
echo ""
echo "Usage : source setup_all.bash [--local]"
echo ""

CURRENT_PATH="`pwd`"

############ Useful functions ################################
function create_backup() {
    local file_to_backup=$1
    if [ -f "${file_to_backup}" ] && [ ! -f "${file_to_backup}.save" ]; then
        cp ${file_to_backup} ${file_to_backup}.save
    fi
}

function search_and_add() {
    #
    # function to search $rx_grep_1 expression in $file
    # - if not found -> add $line
    # - if found, check if the 'exact' $rx_grep_2 expression is in the $file
    # --- if not found -> apply $rx_sed expression to the $file
    # --- else -> nothing to add, everything is in the $file
    #
    local file=$1
    local rx_grep_1=$2
    local rx_grep_2=$3
    local rx_sed=$4
    local task_name=$5
    local line=$6

    local is_found=`cat $file | grep -iE "$rx_grep_1"`
    if [ "$is_found" == "" ]; then
        # rx_grep_1 is not found
        echo "$line" >> $file
        echo "=> Appended $task_name to the file $file"
    else
        # rx_grep_1 is found
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

########### Setup aliases ################################

alias_file="${HOME}/.bash_aliases"
create_backup ${alias_file}

############ alias 'fkart' ################################
echo ""
echo "Setup alias 'fkart'"

alias_cmd="alias fkart=\"cd ${CURRENT_PATH}; source devel/setup.bash\""

if [ -f "$alias_file" ]; then

    search_and_add \
                $alias_file \
                "^alias fkart=\"cd .+; source devel/setup.bash\"" \
                "^alias fkart=\"cd ${CURRENT_PATH}; source devel/setup.bash\"" \
                "/^alias fkart=\"cd .*; source devel\/setup.bash\"/ s/^/#/" \
                "'fkart' alias" \
                "${alias_cmd}"
else
    echo "$alias_cmd" > $alias_file
    echo "=> Created new file ~/.bash_aliases with 'fkart' alias"
fi

############ alias 'save_gmap' ################################
echo ""
echo "Setup alias 'save_gmap'"

alias_cmd="alias save_gmap=\"rosrun map_server map_saver -f ${CURRENT_PATH}/src/futurakart/futurakart_2dnav/maps/_map\""

if [ -f "$alias_file" ]; then

    search_and_add \
                $alias_file \
                "^alias save_gmap=\"rosrun map_server map_saver -f .+\"" \
                "^alias save_gmap=\"rosrun map_server map_saver -f ${CURRENT_PATH}/src/futurakart/futurakart_2dnav/maps/_map\"" \
                "/^alias save_gmap=\"rosrun map_server map_saver -f .*\"/ s/^/#/" \
                "'save_gmap' alias" \
                "${alias_cmd}"
else
    echo "$alias_cmd" > $alias_file
    echo "=> Created new file ~/.bash_aliases with 'save_gmap' alias"
fi

############ alias 'src' ################################
echo ""
echo "Setup alias 'src'"

alias_cmd="alias src=\"source devel/setup.bash\""

if [ -f "$alias_file" ]; then

    search_and_add \
                $alias_file \
                "^alias src=\".+\"" \
                "^alias src=\"source devel/setup.bash\"" \
                "/^alias src=\".*\"/ s/^/#/" \
                "'src' alias" \
                "${alias_cmd}"
else
    echo "$alias_cmd" > $alias_file
    echo "=> Created new file ~/.bash_aliases with 'save_gmap' alias"
fi





############ Setup ROS_MASTER_URI, ROS_HOSTNAME ################################


bashrc_file="${HOME}/.bashrc"

if [ -n "$1" ] && [ "$1" == "--local" ]; then
    ros_master_uri="ROS_MASTER_URI=http://localhost:11311"
    ros_hostname="ROS_HOSTNAME=localhost"
else
    exec 5< ros.conf
    read ros_master_uri <&5
    read ros_hostname <&5        
fi


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
source ${bashrc_file}
echo ""
echo "=> ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "=> ROS_HOSTNAME=${ROS_HOSTNAME}"
