#!/usr/bin/python2

#
# Script to connect the RPi and launch vision.launch file
#

# Python
import sys
import subprocess
import signal


def parse_args(argv):
    ip = None
    username = None
    port = 22
    path = None
    verbose = False
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
        elif splt[0] == "verbose":
            verbose = True if splt[1] == "true" else False

    assert ip is not None and username is not None and path is not None, "Failed to parse given arguments : %s " % argv
    return ip, username, port, path, verbose


def handle_signint(*args, **kwargs):
    # Nothing to do
    pass


if __name__ == "__main__":

    # Script should be called with 5 arguments:
    # [script.py, ssh_ip, ssh_username, '__name:=...', '__log:=...']
    assert len(sys.argv) > 1, "Usage: script.py ip:=1.2.3.4 username:=temp path:=~/futurakart_ws [port:=22]"

    print "Start 'start_vision' node : communicate on ssh *vision* part RPi"
    ip, username, port, path, verbose = parse_args(sys.argv[1:])

    # Need to catch sigint otherwise node is not properly killed
    signal.signal(signal.SIGINT, handle_signint)

    program = ["ssh", "-T", "-o", "VerifyHostKeyDNS no", "%s@%s" % (username, ip), "-p %s" % port]
    proc = subprocess.Popen(program, stdin=None if not verbose else subprocess.PIPE)
    # proc.communicate("cd %s; ls; ping www.google.com" % path)
    proc.communicate("cd %s; source devel/setup.bash; roslaunch futurakart_base vision.launch" % path)
    returncode = proc.wait()
    print "Node 'start_vision' is stopped"
