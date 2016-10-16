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
    # rx_grep_1 = not-exact or exact expression to search. E.g. rx_grep_1="var=.+/path/to/file" or rx_grep_1="var=/exact/path/to/file"
    # rx_grep_2 = exact expression to search (if rx_grep_1 is not exact) or "". E.g rx_grep_2="var=/exact/path/to/file"
    # rx_sed = sed not-exact expression to use to comment found line in the file or "". E.g rx_sed="/^var=.*\/path\/to\/file/ s/^/#/"
    #
    # function to search $rx_grep_1 expression in $file
    # - if not found -> add $line
    # - if found, check if the 'exact' $rx_grep_2 expression is in the $file.
    # --- if not found -> apply $rx_sed expression to the $file. If $rx_sed is "" do not apply sed expression
    # --- else -> nothing to add, everything is in the $file
    # - if found, and if $rx_grep_2 = "" -> nothing to add, everything is in the $file
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
        if [ ! "$rx_grep_2" == "" ]; then
            local is_found=`echo $is_found | grep -iE "$rx_grep_2"`
            if [ "$is_found" == "" ]; then
                # comment previous and append new one
                if [ ! "$rx_sed" == "" ]; then
                    sed -i "$rx_sed" $file
                fi
                echo "$line" >> $file
                echo "=> Comment previous value and appended $task_name to the file $file"
            else
                echo "=> $task_name is already defined"
            fi
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
                "^${alias_cmd}" \
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
                "^${alias_cmd}" \
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
                "^${alias_cmd}" \
                "/^alias src=\".*\"/ s/^/#/" \
                "'src' alias" \
                "${alias_cmd}"
else
    echo "$alias_cmd" > $alias_file
    echo "=> Created new file ~/.bash_aliases with 'save_gmap' alias"
fi

############ Write to bashrc ################################

bashrc_file="${HOME}/.bashrc"
create_backup ${bashrc_file}

############ Source from generic_ws ################################

echo ""
echo "Setup source from generic_ws"

# Search for generic_ws at the same root as futurakart_ws
generic_ws_path=${CURRENT_PATH}/../generic_ws

if [ -d "$generic_ws_path" ]; then
    pushd $generic_ws_path
        _current_path="`pwd`"
        source_gen_cmd="if [ -f ${_current_path}/devel/setup.bash ]; then source ${_current_path}/devel/setup.bash; fi"

        search_and_add \
            $bashrc_file \
            "^if \[ -f .+/generic_ws/devel/setup.bash \]; then source .+/generic_ws/devel/setup.bash; fi" \
            "if \[ -f ${_current_path}/devel/setup.bash \]; then source ${_current_path}/devel/setup.bash; fi" \
            "/^if \[ -f .*\/generic_ws\/devel\/setup.bash \]; then source .*\/generic_ws\/devel\/setup.bash; fi/ s/^/#/" \
            "'source from generic_ws'" \
            "${source_gen_cmd}"


    popd
else
    echo "WARN : Path to generic_ws workspace is not found !"
    echo "WARN : Failed to setup automatic source from generic_ws"
fi




############ Setup ROS_MASTER_URI, ROS_HOSTNAME ################################



if [ -n "$1" ] && [ "$1" == "--local" ]; then
    ros_master_uri="ROS_MASTER_URI=http://localhost:11311"
    ros_hostname="ROS_HOSTNAME=localhost"
else

    if [ ! -f "ros.conf" ]; then
        echo "INFO : Copy ros.conf.example to ros.conf"
        cp ros.conf.example ros.conf
    fi
    exec 5< ros.conf
    read ros_master_uri <&5
    read ros_hostname <&5
fi


echo ""
echo "Apply ros.conf : $ros_master_uri $ros_hostname"

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
