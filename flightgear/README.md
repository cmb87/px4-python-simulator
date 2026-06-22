# Starts Flightgear

Download the flightgear appImage and place it here.

## Known Errors:


In case of 

    sudo  bash start_stream.sh 
    Setting pipeline to PAUSED ...
    Pipeline is live and does not need PREROLL ...
    Pipeline is PREROLLED ...
    Setting pipeline to PLAYING ...
    New clock: GstSystemClock
    Redistribute latency...
    X Error of failed request:  BadMatch (invalid parameter attributes)
    Major opcode of failed request:  130 (MIT-SHM)
    Minor opcode of failed request:  4 (X_ShmGetImage)
    Serial number of failed request:  324
    Current serial number in output stream:  324

Solution: You must have touch or changed the window size. Please restart everysthing!