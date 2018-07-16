import psutil

PROCNAME = "jaco2moveit.py"

for proc in psutil.process_iter():
    print proc
    # check whether the process name matches
    if proc.name == "python":
        print "Killed"
        proc.kill()
        break
