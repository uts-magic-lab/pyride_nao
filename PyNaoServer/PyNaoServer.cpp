/*
 *  PyNaoServer.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 28/08/09.
 *  Copyright 2009 Galaxy Network. All rights reserved.
 *
 */

#include <sys/time.h>
#include <sys/select.h>
#include <stdio.h>

#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

#include <alproxies/alaudiodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <althread/alcriticalsection.h>

#include <PyRideCommon.h>
#include <PythonServer.h>

#include "PyNaoServer.h"
#include "NaoProxyManager.h"
#include "AppConfigManager.h"
#include "PyNAOModule.h"

PYRIDE_LOGGING_DECLARE( "/home/nao/log/tin.log" );

namespace pyride {

static const int kMaxAudioSamples = 16384;
static const float kHFOV = 60.97;
static const float kVFOV = 34.80;

void * videograb_thread( void * controller )
{
  ((NaoCam *)controller)->continueProcessing();
  return NULL;
}
  
NaoCam::NaoCam( boost::shared_ptr<ALBroker> pBroker, const std::string & pName ) :
  broker_( pBroker ),
  procThread_( (pthread_t)NULL ),
  gvmName_( "" ),
  takeSnapShot_( false )
{
  pthread_attr_init( &threadAttr_ );
  pthread_attr_setinheritsched( &threadAttr_, PTHREAD_EXPLICIT_SCHED );
  pthread_attr_setschedpolicy( &threadAttr_, SCHED_OTHER );
  devInfo_.deviceID = "NAOCAM01";
  devInfo_.deviceName = devInfo_.deviceLabel = "NAO Head Cam";
  devInfo_.index = 0;
  devInfo_.shouldBeActive = true;
}

NaoCam::~NaoCam()
{
  pthread_attr_destroy( &threadAttr_ ); 
}

bool NaoCam::initDevice()
{
  if (isInitialised_) {
    return isInitialised_;
  }

  try {
    videoProxy_ = boost::shared_ptr<ALVideoDeviceProxy>(new ALVideoDeviceProxy( broker_ ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALVideoDevice.\n");
    videoProxy_.reset();
    return isInitialised_;
  }
  
  std::string GVMName = "Controller_GVM";
  
  try {
    gvmName_ = videoProxy_->subscribeCamera( GVMName, AL::kTopCamera, AL::kVGA, kYUV422InterlacedColorSpace,
                                      /*kRGBColorSpace,*/ 10 );
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not register GVM to ALVideoDevice.\n" );
    videoProxy_.reset();
    return isInitialised_;
  }
  
  if (!this->getDefaultVideoSettings()) {
    return isInitialised_;
  }
 
  INFO_MSG( "camera resolution = %d\n", videoProxy_->getResolution( gvmName_ ) ); 
  packetStamp_ = 0;
  clientNo_ = 0;
  isStreaming_ = false;
  isInitialised_ = true;

  INFO_MSG( "Nao camera is successfully initialised.\n" );
  return isInitialised_;
}

void NaoCam::finiDevice()
{
  if (!isInitialised_) {
    return;
  }

  this->finiWorkerThread();

  if (videoProxy_) {
    videoProxy_->unsubscribe( gvmName_ );
    videoProxy_.reset();
  }

  isInitialised_ = false;
}

bool NaoCam::initWorkerThread()
{
  if (pthread_create( &procThread_, &threadAttr_, videograb_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to grab Nao camera images.\n" );
    return false;
  }
  return true;
}

void NaoCam::finiWorkerThread()
{
  isStreaming_ = false;
  
  if (procThread_) {
    pthread_join( procThread_, NULL ); // allow thread to exit
    procThread_ = (pthread_t)NULL;
  }
}

void NaoCam::continueProcessing()
{
  ALImage * results;
  
  //int width, height, nbLayers, colorSpace, seconds = 0;
  int rawDataSize = 0;
  unsigned char * rawData = NULL;
  //struct timeval pr1, pr2;
  //long wait;

  while (isStreaming_) {
    //gettimeofday( &pr1, NULL );
    
    results = (ALImage *)(videoProxy_->getDirectRawImageLocal( gvmName_ ));
    if (results) {
      rawData = (unsigned char *)(results->getData());
      rawDataSize = results->getSize();
      processAndSendImageData( rawData, rawDataSize, RAW );
      if (takeSnapShot_) {
        saveToJPEG( rawData, rawDataSize, RAW );
        takeSnapShot_ = false;
      }
    }
    videoProxy_->releaseDirectRawImage( gvmName_ );
  }
}

void NaoCam::takeSnapshot( const VideoDeviceDataHandler * dataHandler )
{
  dataHandler_ = (VideoDeviceDataHandler *)dataHandler;

  if (isStreaming_) {
    takeSnapShot_ = true;
  }
  else {
    ALImage * results = (ALImage *)(videoProxy_->getDirectRawImageLocal( gvmName_ ));
    if (results) {
      unsigned char * rawData = (unsigned char *)(results->getData());
      int rawDataSize = results->getSize();
      saveToJPEG( rawData, rawDataSize, RAW );
    }
    videoProxy_->releaseDirectRawImage( gvmName_ );
  }
}

bool NaoCam::setCameraParameter( int pid, int value )
{
  if (pid < -1 || pid > 34) {
    ERROR_MSG( "PyNaoServer: Invalid camera parameter %d.\n", pid );
    return false;
  }
  switch (pid) {
    case -1:
      return videoProxy_->setAllCameraParametersToDefault( gvmName_ );
    case 3:
    case 14:
    case 15:
    case 24:
      ERROR_MSG( "PyNaoServer: Disabled camera parameter %d.\n", pid );
      return false;
  }
  if (value < 0) {
    return videoProxy_->setCameraParameterToDefault( gvmName_, pid );
  }
  return videoProxy_->setCameraParameter( gvmName_, pid, value );
}

bool NaoCam::getDefaultVideoSettings()
{
  vSettings_.fps = 15;
  vSettings_.format = RGB;
  vSettings_.resolution = 2; // 640x480
  vSettings_.reserved = 0;
  
  this->setProcessParameters();
  
  return true;
}

bool PyNaoServer::initDevice()
{
  if (isInitialised_) {
    return isInitialised_;
  }

  try {
    audioDevice->callVoid( "setClientPreferences", getName(),
                          48000, (int)ALLCHANNELS, 0 );
  }
  catch (const std::exception &error) {
    ERROR_MSG( "PyNaoServer: Could not set parameters to audio device.\n");
    return isInitialised_;
  }
  packetStamp_ = 0;
  clientNo_ = 0;
  isStreaming_ = false;
  isInitialised_ = true;
  aSettings_.reserved = (char)audioDevice->call<int>( "getOutputVolume" );
  INFO_MSG( "Nao audio device is successfully initialised.\n" );
  return isInitialised_;

}

void PyNaoServer::finiDevice()
{
  if (!isInitialised_) {
    return;
  }
  
  this->finiWorkerThread();

  isInitialised_ = false;

}

bool PyNaoServer::initWorkerThread()
{
  try {
    if (audioBuffer_)
      delete [] audioBuffer_; // reset buffer just in case
    
    audioBuffer_ = new AL_SOUND_FORMAT[kMaxAudioSamples];
    this->startDetection();
  }
  catch (const std::exception &error) {
    ERROR_MSG( "Unable to subscribe to audio capturing!\n" );
    return false;
  }
  return true;
}

void PyNaoServer::finiWorkerThread()
{
  isStreaming_ = false;

  try {
    this->stopDetection();
    if (audioBuffer_) {
      delete [] audioBuffer_;
      audioBuffer_ = NULL;
    }
  }
  catch (const std::exception &error) {
    //ERROR_MSG( "Unable to unsubscribe to audio capturing!\n" );
  }
}

void PyNaoServer::process( const int &pNbOfInputChannels, const int &pNbrSamples,
                       const AL_SOUND_FORMAT *pDataInterleaved, const AL::ALValue &pTimeStamp )
{
  if (!audioBuffer_)
    return;
  
  //memcpy( audioBuffer_, pDataInterleaved, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels );
  
  //char nofSkippedChannels = 3;
  
  const AL_SOUND_FORMAT * iterAudioDataSource = pDataInterleaved;
  const AL_SOUND_FORMAT * iterAudioDataSourceEnd = pDataInterleaved+pNbrSamples*pNbOfInputChannels;
  
  AL_SOUND_FORMAT * iterAudioDataSelectedChannel = audioBuffer_;
  // take the 1st left channel
  while (iterAudioDataSource < iterAudioDataSourceEnd) {
    (*iterAudioDataSelectedChannel++) = (*iterAudioDataSource++);
    iterAudioDataSource += 3; //nofSkippedChannels;
  }
  
  //printf( "captured %d audio samples data size %d nofchan %d\n", pNbrSamples, sizeof( AL_SOUND_FORMAT ) * pNbrSamples*pNbOfInputChannels, pNbOfInputChannels );
  this->processAndSendAudioData( audioBuffer_, pNbrSamples );
}

// implementation of PyNaoServer
PyNaoServer::PyNaoServer( boost::shared_ptr<ALBroker> pBroker, const std::string & pName ) :
  ALSoundExtractor( pBroker, pName ),
  AudioDevice( 1 ),
  audioBuffer_( NULL ),
  callbackMutex_( ALMutex::createALMutex() )
{
  setModuleDescription( "This is a server module for TIN." );
  functionName( "onRightBumperPressed", getName(), "Method called when the right bumper is pressed.");
  BIND_METHOD( PyNaoServer::onRightBumperPressed );
  functionName("onLeftBumperPressed", getName(), "Method called when the left bumper is pressed.");
  BIND_METHOD( PyNaoServer::onLeftBumperPressed );  functionName("onFrontTactilTouched", getName(), "Method called when the head front tactile is touched.");

  BIND_METHOD( PyNaoServer::onFrontTactilTouched );
  functionName("onMiddleTactilTouched", getName(), "Method called when the head middle tactile is touched.");
  BIND_METHOD( PyNaoServer::onMiddleTactilTouched );
  functionName("onRearTactilTouched", getName(), "Method called when the head rear tactile is touched.");
  BIND_METHOD( PyNaoServer::onRearTactilTouched );

  functionName("onRightHandBackTouched", getName(), "Method called when the back tactile of the right hand is touched.");
  BIND_METHOD( PyNaoServer::onRightHandBackTouched );
  functionName("onRightHandLeftTouched", getName(), "Method called when the left tactile of the right hand is touched.");
  BIND_METHOD( PyNaoServer::onRightHandLeftTouched );
  functionName("onRightHandRightTouched", getName(), "Method called when the right tactile of the right hand is touched.");
  BIND_METHOD( PyNaoServer::onRightHandRightTouched );
  functionName("onLeftHandBackTouched", getName(), "Method called when the back tactile of the left hand is touched.");
  BIND_METHOD( PyNaoServer::onLeftHandBackTouched );
  functionName("onLeftHandLeftTouched", getName(), "Method called when the left tactile of the left hand is touched.");
  BIND_METHOD( PyNaoServer::onLeftHandLeftTouched );
  functionName("onLeftHandRightTouched", getName(), "Method called when the right tactile of the left hand is touched.");
  BIND_METHOD( PyNaoServer::onLeftHandRightTouched );

  functionName("onSingleChestButtonPressed", getName(), "Method called when the chest button pressed once.");
  BIND_METHOD( PyNaoServer::onSingleChestButtonPressed );
  functionName("onDoubleChestButtonPressed", getName(), "Method called when the chest button pressed twice.");
  BIND_METHOD( PyNaoServer::onDoubleChestButtonPressed );
  functionName("onTripleChestButtonPressed", getName(), "Method called when the chest button pressed three times.");
  BIND_METHOD( PyNaoServer::onTripleChestButtonPressed );
  functionName("onBatteryPowerPlugged", getName(), "Method called when the battery charger is plugged or unplugged.");
  BIND_METHOD( PyNaoServer::onBatteryPowerPlugged );
  functionName("onBatteryChargeChanged", getName(), "Method called when a change in battery level.");
  BIND_METHOD( PyNaoServer::onBatteryChargeChanged );
}

void PyNaoServer::init()
{
  PYRIDE_LOGGING_INIT;

  NaoCam * topCam = new NaoCam( getParentBroker(), getName() );
  if (topCam->initDevice()) {
    naoCams_.push_back( topCam );
  }

  if (this->initDevice()) {
    naoAudio_.push_back( this );
  }

  try {
    memoryProxy_ = boost::shared_ptr<ALMemoryProxy>(new ALMemoryProxy( getParentBroker() ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALMemory.\n");
    memoryProxy_.reset();
  }
  
  NaoProxyManager::instance()->initWithBroker( getParentBroker(), memoryProxy_ );
  ServerDataProcessor::instance()->init( naoCams_, naoAudio_ );
  ServerDataProcessor::instance()->addCommandHandler( this );
  AppConfigManager::instance()->loadConfigFromFile( DEFAULT_CONFIGURATION_FILE );
  ServerDataProcessor::instance()->setClientID( AppConfigManager::instance()->clientID() );
  ServerDataProcessor::instance()->setDefaultRobotInfo( NAO, AppConfigManager::instance()->startPosition() );
  
  PythonServer::instance()->init( AppConfigManager::instance()->enablePythonConsole(), PyNAOModule::instance() );
  ServerDataProcessor::instance()->discoverConsoles();

  if (memoryProxy_) {
    INFO_MSG( "Nao memory proxy is successfully initialised.\n" );
    /* subscribe to sensor events */
    memoryProxy_->subscribeToEvent( "RightBumperPressed", "PyNaoServer", "onRightBumperPressed" );
    memoryProxy_->subscribeToEvent( "LeftBumperPressed", "PyNaoServer", "onLeftBumperPressed" );

    memoryProxy_->subscribeToEvent( "FrontTactilTouched", "PyNaoServer", "onFrontTactilTouched" );
    memoryProxy_->subscribeToEvent( "MiddleTactilTouched", "PyNaoServer", "onMiddleTactilTouched" );
    memoryProxy_->subscribeToEvent( "RearTactilTouched", "PyNaoServer", "onRearTactilTouched" );

    memoryProxy_->subscribeToEvent( "HandRightBackTouched", "PyNaoServer", "onRightHandBackTouched" );
    memoryProxy_->subscribeToEvent( "HandRightLeftTouched", "PyNaoServer", "onRightHandLeftTouched" );
    memoryProxy_->subscribeToEvent( "HandRightRightTouched", "PyNaoServer", "onRightHandRightTouched" );
    memoryProxy_->subscribeToEvent( "HandLeftBackTouched", "PyNaoServer", "onLeftHandBackTouched" );
    memoryProxy_->subscribeToEvent( "HandLightLeftTouched", "PyNaoServer", "onLeftHandLeftTouched" );
    memoryProxy_->subscribeToEvent( "HandLightRightTouched", "PyNaoServer", "onLeftHandRightTouched" );

    memoryProxy_->subscribeToEvent( "ALChestButton/SimpleClickOccurred", "PyNaoServer", "onSingleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/DoubleClickOccurred", "PyNaoServer", "onDoubleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "ALChestButton/TripleClickOccurred", "PyNaoServer", "onTripleChestButtonPressed" );
    memoryProxy_->subscribeToEvent( "BatteryPowerPluggedChanged", "PyNaoServer", "onBatteryPowerPlugged" );
    memoryProxy_->subscribeToEvent( "BatteryChargeChanged", "PyNaoServer", "onBatteryChargeChanged" );
  }
}

void PyNaoServer::fini()
{
  this->notifySystemShutdown();

  for (int i = 0; i < naoCams_.size(); i++) {
    VideoDevice * naocam = naoCams_.at( i );
    naocam->finiDevice();
    delete naocam;
  }
  naoCams_.clear();
  
  this->finiDevice();
  naoAudio_.clear();

  NaoProxyManager::instance()->fini();
  AppConfigManager::instance()->fini();
  ServerDataProcessor::instance()->fini();

  /*
  if (memoryProxy_) {
    memoryProxy_->unsubscribeToEvent( "RightBumperPressed", getName() );
    memoryProxy_->unsubscribeToEvent( "LeftBumperPressed", getName() );
    memoryProxy_->unsubscribeToEvent( "ALSentinel/SimpleClickOccured", getName() );
    memoryProxy_->unsubscribeToEvent( "ALSentinel/DoubleClickOccured", getName() );
    memoryProxy_->unsubscribeToEvent( "ALSentinel/TripleClickOccured", getName() );
    memoryProxy_.reset();
  }*/
}

bool PyNaoServer::executeRemoteCommand( PyRideExtendedCommand command, 
                                            const unsigned char * optionalData,
                                            const int optionalDataLength )
{
  // implement the routine to handle commands defined in PyRideExtendedCommand
  // in PyRideCommon.h
  // for example:
  bool status = true;
  switch (command) {
    case SPEAK:
    {
      float volume = *((float *)optionalData);
      //DEBUG_MSG( "received volume %f\n", volume );
      char * text = (char *)optionalData + sizeof( float );
      NaoProxyManager::instance()->sayWithVolume( std::string( text, optionalDataLength - sizeof( float ) ), volume );
    }
      break;
    case HEAD_MOVE_TO:
    {
      float newHeadYaw = *((float *)optionalData);
      float newHeadPitch = *((float *)optionalData+1);
      newHeadYaw = newHeadYaw * kHFOV * kDegreeToRAD;
      newHeadPitch = newHeadPitch * kVFOV * kDegreeToRAD;
      NaoProxyManager::instance()->moveHeadTo( newHeadYaw, newHeadPitch, true );
      NaoProxyManager::instance()->setHeadStiffness( 0.0 );
    }
      break;
    case UPDATE_BODY_POSE:
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      NaoProxyManager::instance()->updateBodyPose( newPose );
    }
      break;
    case UPDATE_AUDIO_SETTINGS:
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      int volume;
      memcpy( &volume, dataPtr, sizeof( int ) );
      NaoProxyManager::instance()->setAudioVolume( volume );
    }
      break;    
    default:
      status = false;
      break;
  }
  return status;
}

void PyNaoServer::cancelCurrentOperation()
{
  NaoProxyManager::instance()->sayWithVolume( "Emergency Stop!" );
}

PyNaoServer::~PyNaoServer()
{
  this->fini();
}

#pragma callback functions from alproxies.

/** @name Event Callback Functions
 *
 */
/**@{*/
/*! \typedef onBumperPressed(side)
 *  \memberof PyNAO.
 *  \brief Callback function when one of NAO's bumpers is pressed.
 *  \param str side: side = "right" for the right bumper or  "left" for the left bumper.
 *  \return None.
 */
void PyNaoServer::onRightBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "RightBumperPressed" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    arg = Py_BuildValue( "(s)", "right" );

    PyNAOModule::instance()->invokeCallback( "onBumperPressed", arg );
    Py_DECREF( arg );
    
    PyGILState_Release( gstate );
  }
}

void PyNaoServer::onLeftBumperPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "LeftBumperPressed" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    arg = Py_BuildValue( "(s)", "left" );
    
    PyNAOModule::instance()->invokeCallback( "onBumperPressed", arg );
    Py_DECREF( arg );
    
    PyGILState_Release( gstate );
  }
}

/*! \typedef onHeadTactileTouched(side)
 *  \memberof PyNAO.
 *  \brief Callback function when one of NAO's head tactile sensor is touched.
 *  \param str side: side = "front" for the front tactile,  "middle" for the middle tactile and "rear" from the back tactile.
 *  \return None.
 */
void PyNaoServer::onFrontTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "FrontTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "front" );

    PyNAOModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyNaoServer::onMiddleTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "MiddleTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "middle" );

    PyNAOModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void PyNaoServer::onRearTactilTouched()
{
  ALCriticalSection section( callbackMutex_ );

  /**
   * Check that the bumper is pressed.
   */
  float stat =  memoryProxy_->getData( "RearTactilTouched" );
  if (stat  > 0.5f) {
    PyObject * arg = NULL;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    arg = Py_BuildValue( "(s)", "rear" );

    PyNAOModule::instance()->invokeCallback( "onHeadTactileTouched", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

/*! \typedef onRightHandTouched(side, status)
 *  \memberof PyNAO.
 *  \brief Callback function when one of NAO's right hand tactile sensor is touched.
 *  \param str side. side = "back" for the back tactile,  "right" for the right tactile and "left" from the left tactile.
 *  \param bool status. True == touch on, False == touch off.
 *  \return None.
 */
void PyNaoServer::onRightHandBackTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandRightBackTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "back", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onRightHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyNaoServer::onRightHandLeftTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandRightLeftTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "left", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onRightHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyNaoServer::onRightHandRightTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandRightRightTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "right", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onRightHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onLeftHandTouched(side, status)
 *  \memberof PyNAO.
 *  \brief Callback function when one of NAO's left hand tactile sensor is touched.
 *  \param str side. side = "back" for the back tactile,  "right" for the right tactile and "left" from the left tactile.
 *  \param str status. status = "on" for touch on or "off" for touch off.
 *  \return None.
 */
void PyNaoServer::onLeftHandBackTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandLeftBackTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "back", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onLeftHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyNaoServer::onLeftHandLeftTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandLeftLeftTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "left", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onLeftHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void PyNaoServer::onLeftHandRightTouched()
{
  ALCriticalSection section( callbackMutex_ );

  float stat =  memoryProxy_->getData( "HandLeftRightTouched" );

  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(sO)", "right", (stat > 0.5f ? Py_True : Py_False) );

  PyNAOModule::instance()->invokeCallback( "onLeftHandTouched", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onChestButtonPressed(presses)
 *  \memberof PyNAO.
 *  \brief Callback function when the chest button is pressed.
 *  \param int presses. Number of presses occurred.
  *  \return None.
 */
void PyNaoServer::onSingleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", 1 );
  
  PyNAOModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

void PyNaoServer::onDoubleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", 2 );
  
  PyNAOModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

void PyNaoServer::onTripleChestButtonPressed()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", 3 );
  
  PyNAOModule::instance()->invokeCallback( "onChestButtonPressed", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

/*! \typedef onPowerPluggedChange(is_plugged)
 *  \memberof PyNAO.
 *  \brief Callback function when NAO change between battery power or main power.
 *  \param bool is_plugged. True == plugged into main, False == on battery power.
  *  \return None.
 */
void PyNaoServer::onBatteryPowerPlugged()
{
  ALCriticalSection section( callbackMutex_ );
  
  /**
   * Check that the bumper is pressed.
   */
  bool isplugged =  memoryProxy_->getData( "BatteryPowerPluggedChanged" );
  PyObject * arg = NULL;
    
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(O)", isplugged ? Py_True : Py_False );

  PyNAOModule::instance()->invokeCallback( "onPowerPluggedChange", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

/*! \typedef onBatteryChargeChange(bat_percent, is_discharging)
 *  \memberof PyNAO.
 *  \brief Callback function when the NAO battery status has changed.
 *  \param int bat_percent. The remaining battery percentage within [0..100].
 *  \param bool is_discharging. True == the battery is discharging, False == the batter is charging.
  *  \return None.
 */
void PyNaoServer::onBatteryChargeChanged()
{
  ALCriticalSection section( callbackMutex_ );
  
  int batpercent =  memoryProxy_->getData( "BatteryChargeChanged" );
  bool discharging = memoryProxy_->getData( "BatteryDisChargingFlagChanged" );

  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(iO)", batpercent, discharging ? Py_True : Py_False );
  
  PyNAOModule::instance()->invokeCallback( "onBatteryChargeChange", arg );

  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

/*! \typedef onSystemShutdown()
 *  \memberof PyNAO.
 *  \brief Callback function when the NAO is shutting down.
 *  \return None.
 *  \note Currently (v2.1.4) NaoQi does not exit correctly when NAO is shutting down using the chest button. Init script needs update.
 */
void PyNaoServer::notifySystemShutdown()
{
  INFO_MSG( "PyNaoServer is shutting down..\n" );

  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyNAOModule::instance()->invokeCallback( "onSystemShutdown", arg );
  
  PyGILState_Release( gstate );
}
/**@}*/

} // namespace pyride
