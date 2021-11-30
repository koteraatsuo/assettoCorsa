################################################################################
# Never Eat Yellow Snow APPS
# 
# workarounds for some defects in ac python api
#
# Changelog:
#
# - Initial version (partly moved from map_display)
#     
################################################################################

import sys
from map_display_lib import sim_info
import time
import ac
import acsys

session_type = None
lastCallTime = None

def acGetSessionType():
    global session_type, lastCallTime
    currTime = time.clock()
    # this function should really be implemented in the python API, for now we get this info from 
    # the shared memoryview
    # Because the performance of the shared mem settings is "suspicious", we interpret the shared mem
    # only each 1000th frame. 
    if lastCallTime is None:
        lastCallTime = currTime - 5.
    if currTime - lastCallTime > 1.:
        session_type = None
    lastCallTime = currTime
    if session_type is None:
        sim_info_obj = sim_info.SimInfo()
        sim_info_session_types = {
            sim_info.AC_UNKNOWN : 'unknown',
            sim_info.AC_PRACTICE : 'practice',
            sim_info.AC_QUALIFY : 'qualify',
            sim_info.AC_RACE : 'race',
            sim_info.AC_HOTLAP : 'hotlap',
            sim_info.AC_TIME_ATTACK : 'timeattack',
            sim_info.AC_DRIFT : 'drift',
            sim_info.AC_DRAG : 'drag',
        }
        if not sim_info_obj.graphics.session in sim_info_session_types:
            sim_info_session_types[sim_info_obj.graphics.session] = 'invalid/unknown'
        #ac.log("session_type is %d (%s)" % (sim_info_obj.graphics.session, sim_info_session_types[sim_info_obj.graphics.session]))
        session_type = sim_info_session_types[sim_info_obj.graphics.session]
    return session_type

lastLapStatus = {}
lastSessionType = None
lastEgoLaps = None
def acGetLapCount(carid):
    global lastLapStatus, lastSessionType, lastEgoLaps
    # workaround for providing the lap counts in multiplayer
    currSessionType = acGetSessionType()
    currEgoLaps = ac.getCarState(0, acsys.CS.LapCount)
    if lastEgoLaps is None:
        lastEgoLaps = currEgoLaps
    if currSessionType != lastSessionType or lastSessionType is None or lastEgoLaps > currEgoLaps:
        lastLapStatus = {}
    lastEgoLaps = currEgoLaps
    lastSessionType = currSessionType
    splPos = ac.getCarState(carid, acsys.CS.NormalizedSplinePosition)
    if not carid in lastLapStatus:
        if splPos > 0.1:
            lapCnt = -1
        else:
            lapCnt = 0
    else:
        lapCnt = lastLapStatus[carid][1]
        if splPos - lastLapStatus[carid][0] < -0.5:
            lapCnt += 1
    lastLapStatus[carid] = (splPos, lapCnt)
    return lapCnt

prof_stats = []
lastProfLog = None
def profile_call(modulename, func, args):
    global prof_stats, lastProfLog
    t1 = time.clock()
    if lastProfLog is None:
        lastProfLog = t1
    res = func(*args)
    t2 = time.clock()
    prof_stats.append(t2-t1)
    if t2 - lastProfLog > 30.0: 
        ac.log("%s: fps: %5.1f execution times: min=%5.2f ms, max=%5.2f ms, avg=%5.2f" % (
                modulename, 
                len(prof_stats)/(t2-lastProfLog), 
                min(prof_stats)*1000., 
                max(prof_stats)*1000., 
                sum(prof_stats)*1000./len(prof_stats)))
        lastProfLog = t2
        prof_stats = []
    return res