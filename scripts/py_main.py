import PyNAO
import math
import constants
import tinstate
import messenger
import tininfo
import time

from timers import timermanager

myMessenger = None

msgTryTimer = -1
calTryTimer = -1
cktime = time.time()

def userLogon( name ):
  PyNAO.say( '%s has logged on.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, False )

def userLogoff( name ):
  PyNAO.say( '%s has logged off.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, len(PyNAO.listCurrentUsers()) == 0)

def bumperActions( id ):
  global cktime
  if time.time() - cktime <= 60:
    return #ignore

  if id == 'right':
    users = PyNAO.listCurrentUsers()
    if len(users) == 0:
      PyNAO.say( 'No one is telepresent.' )
    else:
      PyNAO.say( 'Following users are telepresent:' )
      for i in users:
        PyNAO.say( '\pau=1000\ %s' % i )
  elif id == 'left':
    if len(PyNAO.listCurrentUsers()) > 0:
      PyNAO.say( 'I will notify the telepresent members to tune in.' )
      PyNAO.updateOperationalStatus( constants.CUSTOM_STATE, 'Need your attention' )

  cktime = time.time()

def remoteCommandActions( cmd, arg ):
  pass

def timerLapsedActions( id ):
  timermanager.onTimerLapsed( id )

def timerActions( id ):
  global myMessenger, msgTryTimer

  if msgTryTimer == id and myMessenger.checkin():
    PyNAO.removeTimer( msgTryTimer )
    msgTryTimer = -1
  else:
    timermanager.onTimerCall( id )

def chestBtnActions( id ):
  global myMessenger, purgeuser
  if id == 1:
    myMessenger.announce()
#do report messages
  elif id == 2:
    myMessenger.purgearchive()
  elif id == 3:
    PyNAO.say( constants.INTRO_TEXT )

def powerPlugChangeActions( isplugged ):
  global myMessenger

  text = ""
  if isplugged:
    text = "I'm on main power."
  else:
    text = "I'm on battery power."

  PyNAO.say( text )

  if myMessenger:
    myMessenger.updatestatus( text )

def batteryChargeChangeActions( batpc, isdischarge ):
  global myMessenger
  
  if batpc < 20 and isdischarge:
    PyNAO.say( "I'm low on battery, please put me back on main power." )

    if myMessenger:
      myMessenger.updatestatus( "I have only %d percent battery power left!" % batpc )

def systemShutdownActions():
  global myMessenger

  myMessenger.checkout()
  PyNAO.say( "I am going off line. Goodbye." )

def main():
  global myMessenger, msgTryTimer

  PyNAO.onUserLogOn = userLogon
  PyNAO.onUserLogOff = userLogoff
  PyNAO.onBumperPressed = bumperActions
  PyNAO.onTimer = timerActions
  PyNAO.onTimerLapsed = timerLapsedActions
  PyNAO.onRemoteCommand = remoteCommandActions
  PyNAO.onChestButtonPressed = chestBtnActions
  PyNAO.onSystemShutdown = systemShutdownActions
  PyNAO.onPowerPluggedChange = powerPlugChangeActions
  PyNAO.onBatteryChargeChange = batteryChargeChangeActions
  
  PyNAO.say( constants.INTRO_TEXT )

  myMessenger = messenger.Messenger()
  if not myMessenger.checkin():
    msgTryTimer = PyNAO.addTimer( 10*60, -1, 10*60 )

