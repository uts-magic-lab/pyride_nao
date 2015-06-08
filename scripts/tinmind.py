import PyNAO
import re
import tininfo

loadedSongs = {}

def respond( question ):
  q = question.strip().lower()

  if 'ip' in q and 'addr' in q:
    return "My IP Address is %s." % PyNAO.getMyIPAddress()
  
  if 'battery' in q and ('how much' in q or 'status' in q):
    (batpc, isplug, ischarging) = PyNAO.getBatteryStatus()
    return "I'm currently %s with %d percent battery power and %s." % (isplug, batpc, ischarging)

  if 'hdt' in q:
    obj = PyNAO.getHDTModule()
    if obj:
      if 'start' in q:
        obj.startDetection()
        return "Start human tracking."
      elif 'stop' in q:
        obj.stopDetection()
        return "Stop human tracking."
      elif 'check' in q or 'observation' in q:
        if not obj.isStreaming:
          return "Start Human detection manually first."
        return obj.latestObservations()
    else:
      return "Human detection module is not online."
  
  m = re.match(r"\W*(play|sing) (?P<song_name>\w+)", q )
  if m:
    try:
      fname = '/home/nao/recordings/%s.mp3' % m.group('song_name')
      if fname not in loadedSongs:
        loadedSongs[fname] = PyNAO.loadAudioFile( fname )
      PyNAO.playAudioID( loadedSongs[fname] )
      return "Playing the song."

    except:
      return "I couldn't find song '%s'." % m.group('song_name') 
    
  return "I don't understand your request." 

def eventAction( event ):
  text = event.text.strip().lower()
  m = re.match(r"(?P<name>\w+).*? birthday", text )

  if m:
    bp = m.group('name')
    PyNAO.say( "%s \pau=500\ are you there? \pau=700\ It's your birthday!" % bp, 1.0, True )
    try:
      if 'birthday' not in loadedSongs:
        loadedSongs['birthday'] = PyNAO.loadAudioFile( '/home/nao/recordings/birthday.mp3' )
      PyNAO.playAudioID( loadedSongs['birthday'], True )
    except:
      pass

    PyNAO.say( "Hurray Hurray. Happy birthday, %s!" % bp )
    event.nofnotices = 0
    return "Happy birthday to %s!" % bp

  if event.where:
    sayString = "%s is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, event.where)
    textString = "'%s' is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, event.where)
  else:
    sayString = "%s is about to start in %d minutes in %s." % (event.text, event.nofnotices*5, tininfo.TiNLocation)
    textString = "'%s' is about to start in %d minutes in %s." % (event.text,event.nofnotices*5, tininfo.TiNLocation)
      
  PyNAO.say( sayString )
  event.nofnotices = event.nofnotices - 1

  return textString
