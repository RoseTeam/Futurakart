#!/usr/bin/python2

#
# Script to connect the RPi and launch vision.launch file
#

# Python
import sys
import subprocess


def parse_args(argv):
    ip = None
    username = None
    port = 22
    path = None
    for arg in argv:
        splt = arg.split(":=")
        if splt[0] == "ip":
            ip = splt[1]
        elif splt[0] == "username":
            username = splt[1]
        elif splt[0] == "port":
            port = splt[1]
        elif splt[0] == "path":
            path = splt[1]

    assert ip is not None and username is not None and path is not None, "Failed to parse given arguments : %s " % argv
    return ip, username, port, path

if __name__ == "__main__":

    # Script should be called with 5 arguments:
    # [script.py, ssh_ip, ssh_username, '__name:=...', '__log:=...']
    assert len(sys.argv) > 1, "Usage: script.py ip:=1.2.3.4 username:=temp path:=~/futurakart_ws [port:=22]"

    ip, username, port, path = parse_args(sys.argv[1:])

    program = ["ssh", "-T", "-o", "VerifyHostKeyDNS no", "%s@%s" % (username, ip), "-p %s" % port]
    proc = subprocess.Popen(program, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate("cd %s; xterm" % path)
    # out, err = proc.communicate("cd %s; source devel/setup.bash; roslaunch futurakart_base vision.launch &" % path)
    print out
    returncode = proc.poll()
    if proc.wait() == 0:
        # everything is OK
        print "Vision part is started"
    else:
        print err
    print "Return code = ", returncode