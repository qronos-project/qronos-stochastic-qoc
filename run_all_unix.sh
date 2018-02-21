#!/bin/sh
# start run_all.m on Linux (should also work on Mac)
# Your system must be set up so that the 'matlab' command called from the shell works correctly.
set -e
which matlab > /dev/null || { echo 'matlab was not found in PATH'; exit 1; }
[ "x$1" = "x--fast" ] && export QRONOS_QOC_CI_SPEEDUP_ENABLED=1 || echo "use --fast to speed up at cost of accuracy (see param_global.m, simulation_speedup_factor";
[ "x$QRONOS_QOC_MATLAB" = "x" ] && { echo "using default MATLAB path."; QRONOS_QOC_MATLAB="$(which matlab)"; }
echo "Matlab executable: QRONOS_QOC_MATLAB=$QRONOS_QOC_MATLAB"
exec $QRONOS_QOC_MATLAB  -nodisplay -nosplash -nodesktop -r "ver, try, run_all, disp('Success.'), exit(0), catch e, disp(e.getReport()), disp('ERROR'), exit(1), end"
