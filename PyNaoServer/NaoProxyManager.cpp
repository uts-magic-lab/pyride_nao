/*
 *  NaoProxyManager.cpp
 *  PyNaoServer
 *
 *  Created by Xun Wang on 16/08/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */
#include <sys/time.h>
#include <PyRideCommon.h>
#include "NaoProxyManager.h"

namespace pyride {

static const long kMotionCommandGapTolerance = 2 * 1000000 / kMotionCommandFreq;

static const char *kNAOBodyJoints[] = {
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw"
};

static const int kNAOJointNo = sizeof( kNAOBodyJoints ) / sizeof( kNAOBodyJoints[0] );

NaoProxyManager * NaoProxyManager::s_pNaoProxyManager = NULL;

void * pulse_thread( void * controller )
{
  ((NaoProxyManager *)controller)->continuePulseChestLED();
  return NULL;
}

void * timeout_thread( void * controller )
{
  ((NaoProxyManager *)controller)->timeoutCheck();
  return NULL;
}

NaoProxyManager::NaoProxyManager() :
  moveInitialised_( false ),
  isChestLEDPulsating_( false ),
  runningThread_( (pthread_t)NULL ),
  timeoutThread_( (pthread_t)NULL )
{
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_, &t_mta );
}

NaoProxyManager::~NaoProxyManager()
{
  pthread_mutex_destroy( &t_mutex_ );
  pthread_mutexattr_destroy( &t_mta );
}

NaoProxyManager * NaoProxyManager::instance()
{
  if (!s_pNaoProxyManager) {
    s_pNaoProxyManager = new NaoProxyManager();
  }
  return s_pNaoProxyManager;
}

void NaoProxyManager::initWithBroker( boost::shared_ptr<ALBroker> broker, boost::shared_ptr<ALMemoryProxy> memoryProxy )
{
  memoryProxy_ = memoryProxy;

  try {
    speechProxy_ = boost::shared_ptr<ALTextToSpeechProxy>(new ALTextToSpeechProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALTextToSpeech.\n");
    speechProxy_.reset();
  }
  if (speechProxy_) {
    INFO_MSG( "Nao text to speech is successfully initialised.\n" );
  }

  try {
    ledProxy_ = boost::shared_ptr<ALLedsProxy>(new ALLedsProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALLeds.\n");
    ledProxy_.reset();
  }
  if (ledProxy_) {
    INFO_MSG( "Nao LED control is successfully initialised.\n" );
  }

  try {
    audioDeviceProxy_ = boost::shared_ptr<ALAudioDeviceProxy>(new ALAudioDeviceProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALAudioDevice.\n");
    audioDeviceProxy_.reset();
  }
  if (audioDeviceProxy_) {
    INFO_MSG( "Nao ALAudioDevice are successfully initialised.\n" );
  }

  try {
    audioPlayerProxy_ = boost::shared_ptr<ALAudioPlayerProxy>(new ALAudioPlayerProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALAudioPlayer.\n");
    audioPlayerProxy_.reset();
  }
  if (audioPlayerProxy_) {
    INFO_MSG( "Nao ALAudioPlayer are successfully initialised.\n" );
  }

  try {
    behaviourManagerProxy_ = boost::shared_ptr<ALBehaviorManagerProxy>(new ALBehaviorManagerProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALBehaviourManager.\n");
    behaviourManagerProxy_.reset();
  }
  if (behaviourManagerProxy_) {
    INFO_MSG( "Nao ALBehaviourManager is successfully initialised.\n" );
  }

  try {
    motionProxy_ = boost::shared_ptr<ALMotionProxy>(new ALMotionProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALMotion.\n");
    motionProxy_.reset();
  }

  if (motionProxy_) {
    moveInitialised_ = false;
    AL::ALValue joints; // make sure we get joint limits in correct order
    jointLimits_.arraySetSize( 26 );
    jointLimits_[0] = motionProxy_->getLimits( "HeadYaw" )[0];
    jointLimits_[1] = motionProxy_->getLimits( "HeadPitch" )[0];
    jointLimits_[2] = motionProxy_->getLimits( "LShoulderPitch" )[0];
    jointLimits_[3] = motionProxy_->getLimits( "LShoulderRoll" )[0];
    jointLimits_[4] = motionProxy_->getLimits( "LElbowYaw" )[0];
    jointLimits_[5] = motionProxy_->getLimits( "LElbowRoll" )[0];
    jointLimits_[6] = motionProxy_->getLimits( "LWristYaw" )[0];
    jointLimits_[7] = motionProxy_->getLimits( "LHipYawPitch" )[0];
    jointLimits_[8] = motionProxy_->getLimits( "LHipRoll" )[0];
    jointLimits_[9] = motionProxy_->getLimits( "LHipPitch" )[0];
    jointLimits_[10] = motionProxy_->getLimits( "LKneePitch" )[0];
    jointLimits_[11] = motionProxy_->getLimits( "LAnklePitch" )[0];
    jointLimits_[12] = motionProxy_->getLimits( "LAnkleRoll" )[0];
    jointLimits_[13] = motionProxy_->getLimits( "RHipYawPitch" )[0];
    jointLimits_[14] = motionProxy_->getLimits( "RHipRoll" )[0];
    jointLimits_[15] = motionProxy_->getLimits( "RHipPitch" )[0];
    jointLimits_[16] = motionProxy_->getLimits( "RKneePitch" )[0];
    jointLimits_[17] = motionProxy_->getLimits( "RAnklePitch" )[0];
    jointLimits_[18] = motionProxy_->getLimits( "RAnkleRoll" )[0];
    jointLimits_[19] = motionProxy_->getLimits( "RShoulderPitch" )[0];
    jointLimits_[20] = motionProxy_->getLimits( "RShoulderRoll" )[0];
    jointLimits_[21] = motionProxy_->getLimits( "RElbowYaw" )[0];
    jointLimits_[22] = motionProxy_->getLimits( "RElbowRoll" )[0];
    jointLimits_[23] = motionProxy_->getLimits( "RWristYaw" )[0];
    jointLimits_[24] = motionProxy_->getLimits( "LHand" )[0];
    jointLimits_[25] = motionProxy_->getLimits( "RHand" )[0];

    INFO_MSG( "Nao Motion is successfully initialised.\n" );
  }

  try {
    postureProxy_ = boost::shared_ptr<ALRobotPostureProxy>(new ALRobotPostureProxy( broker ));
  }
  catch (const ALError& e) {
    ERROR_MSG( "PyNaoServer: Could not create a proxy to ALRobotPosture.\n");
    postureProxy_.reset();
  }
  if (postureProxy_) {
    postureProxy_->goToPosture( "Crouch", 0.6 );
    if (motionProxy_) {
      motionProxy_->rest();
    }
    INFO_MSG( "Nao Robot Posture is successfully initialised.\n" );
  }
}

void NaoProxyManager::sayWithVolume( const std::string & text, float volume, bool toBlock )
{
  if (speechProxy_ && text.length()) {
    if (volume <= 1.0 && volume > 0.0) {
      speechProxy_->setVolume( volume );
    }
    if (toBlock) {
      speechProxy_->say( text );
    }
    else {
      speechProxy_->post.say( text );
    }
  }
}

bool NaoProxyManager::getHeadPos( float & yaw, float & pitch )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    std::vector<float> curHeadPos = motionProxy_->getAngles( names, true );
    yaw = curHeadPos.at( 0 );
    pitch = curHeadPos.at( 1 );
    return true;
  }
  return false;
}

void NaoProxyManager::moveHeadTo( const float yaw, const float pitch, bool relative, float frac_speed )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    AL::ALValue stiff = 1.0f;
    AL::ALValue newHeadPos;

    newHeadPos.arraySetSize( 2 );

    if (relative) {
      std::vector<float> curHeadPos;

      curHeadPos = motionProxy_->getAngles( names, true );

      newHeadPos[0] = clamp( yaw + curHeadPos.at( 0 ), HEAD_YAW );
      newHeadPos[1] = clamp( pitch + curHeadPos.at( 1 ), HEAD_PITCH );
    }
    else {
      newHeadPos[0] = clamp( yaw, HEAD_YAW );
      newHeadPos[1] = clamp( pitch, HEAD_PITCH );
    }
    motionProxy_->setStiffnesses( names, stiff );

    try {
      motionProxy_->angleInterpolationWithSpeed( names, newHeadPos, frac_speed );
    }
    catch (...) {
      ERROR_MSG( "Unable to set angle interpolation to %s", newHeadPos.toString().c_str() );
    }
  }
}

void NaoProxyManager::updateHeadPos( const float yaw, const float pitch, const float speed )
{
  if (motionProxy_) {
    AL::ALValue names = "Head";
    AL::ALValue newHeadPos;
    AL::ALValue stiff = 1.0f;

    newHeadPos.arraySetSize( 2 );

    std::vector<float> curHeadPos;
    motionProxy_->setStiffnesses( names, stiff );

    curHeadPos = motionProxy_->getAngles( names, true );

    newHeadPos[0] = clamp( yaw + curHeadPos.at( 0 ), HEAD_YAW );
    newHeadPos[1] = clamp( pitch + curHeadPos.at( 1 ), HEAD_PITCH );

    float myspeed = (speed > 1.0 || speed < 0.0 ) ? 0.1 : speed; // default to 0.1
    try {
      motionProxy_->setAngles( names, newHeadPos, myspeed );
    }
    catch (...) {
      ERROR_MSG( "Unable to change angles to %s", newHeadPos.toString().c_str() );
    }
  }
}

void NaoProxyManager::setHeadStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Head";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void NaoProxyManager::setBodyStiffness( const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = "Body";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void NaoProxyManager::sit( bool relax )
{
  if (postureProxy_) {
    postureProxy_->goToPosture( relax ? "SitRelax" : "Sit", 0.7 );
    motionProxy_->rest();
    moveInitialised_ = false;
  }
}

void NaoProxyManager::stand( bool init )
{
  if (postureProxy_) {
    postureProxy_->goToPosture( init ? "StandInit" : "Stand", 0.7 );
    moveInitialised_ = false;
  }
}

void NaoProxyManager::crouch()
{
  if (postureProxy_) {
    postureProxy_->goToPosture( "Crouch", 0.7 );
    motionProxy_->rest();
    moveInitialised_ = false;
  }
}

void NaoProxyManager::lyingDown( bool bellyUp )
{
  if (postureProxy_) {
    postureProxy_->goToPosture( bellyUp ? "LyingBack" : "LyingBelly", 0.7 );
    motionProxy_->rest();
    moveInitialised_ = false;
  }
}

bool NaoProxyManager::moveBodyTo( const RobotPose & pose, bool cancelPreviousMove, bool inpost )
{
  if (!motionProxy_) {
    ERROR_MSG( "Unable to initialise walk." );
    return false;
  }
  if (motionProxy_->moveIsActive()) {
    if (cancelPreviousMove) {
      motionProxy_->stopMove();
    }
    else {
      ERROR_MSG( "Unable to issue new work command, robot is moving." );
      return false;
    }
  }
  else if (!moveInitialised_) {
    motionProxy_->wakeUp();
    motionProxy_->moveInit();
    moveInitialised_ = true;
  }
  if (inpost)
    motionProxy_->post.moveTo( pose.x, pose.y, pose.theta );
  else
    motionProxy_->moveTo( pose.x, pose.y, pose.theta );
  return true;
}

void NaoProxyManager::cancelBodyMovement()
{
  if (motionProxy_ && motionProxy_->moveIsActive()) {
    motionProxy_->stopMove();
  }
}

void NaoProxyManager::updateBodyPose( const RobotPose & pose )
{
  if (!motionProxy_)
    return;

  gettimeofday( &cmdTimeStamp_, NULL );

  if (!moveInitialised_) {
    motionProxy_->wakeUp();
    motionProxy_->moveInit();
    moveInitialised_ = true;
  }
  motionProxy_->move( pose.x/3, pose.y/3, pose.theta/2 );
  if (!timeoutThread_) {
    if (pthread_create( &timeoutThread_, NULL, timeout_thread, this ) ) {
      ERROR_MSG( "Unable to create thread to check motion command expiry.\n" );
      timeoutThread_ = (pthread_t)NULL;
      return;
    }
  }
}

void NaoProxyManager::getBodyJointsPos( std::vector<float> & positions,
                      bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    //AL::ALValue names = "Body";

    // skip hands
    AL::ALValue names;
    names.arraySetSize( kNAOJointNo );
    for (int i = 0; i < kNAOJointNo; i++) {
      names[i] = kNAOBodyJoints[i];
    }

    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void NaoProxyManager::getArmJointsPos( bool isLeft, std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void NaoProxyManager::getLegJointsPos( bool isLeft, std::vector<float> & positions,
                     bool useSensor )
{
  positions.clear();
  if (motionProxy_) {
    AL::ALValue names = isLeft ? "LLeg" : "RLeg";
    positions = motionProxy_->getAngles( names, useSensor );
  }
}

void NaoProxyManager::setArmStiffness( bool isLeft, const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = isLeft ? "LArm" : "RArm";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

void NaoProxyManager::setLegStiffness( bool isLeft, const float stiff )
{
  if (motionProxy_ && stiff >= 0.0 && stiff <= 1.0) {
    AL::ALValue names = isLeft ? "LLeg" : "RLeg";
    motionProxy_->setStiffnesses( names, stiff );
  }
}

bool NaoProxyManager::moveArmWithJointPos( bool isLeftArm, const std::vector<float> & positions,
                                           float frac_speed, bool inpost )
{
  if (!motionProxy_)
    return false;

  int pos_size = positions.size();
  if (pos_size != 5) {
    return false;
  }

  //AL::ALValue names = isLeftArm ? "LArm" : "RArm";
  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll",
                                                      "LWristYaw" ) :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll",
                                                      "RWristYaw");

  AL::ALValue angles;

  motionProxy_->setStiffnesses( names, 1.0 );

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], (isLeftArm ? L_SHOULDER_PITCH + i : R_SHOULDER_PITCH + i) );
  }
  try {
    if (inpost)
      motionProxy_->setAngles( names, angles, frac_speed );
    else
      motionProxy_->angleInterpolationWithSpeed( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;
}

bool NaoProxyManager::moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<float> > & trajectory,
                                                 std::vector<float> & times_to_reach, bool inpost )
{
  if (!motionProxy_)
    return false;

  size_t traj_size = trajectory.size();

  if (traj_size <= 0 || traj_size != times_to_reach.size())
    return false;

  //AL::ALValue names = isLeftArm ? "LArm" : "RArm";
  AL::ALValue names = isLeftArm ? AL::ALValue::array( "LShoulderPitch",
                                                      "LShoulderRoll",
                                                      "LElbowYaw",
                                                      "LElbowRoll",
                                                      "LWristYaw" ) :
                                  AL::ALValue::array( "RShoulderPitch",
                                                      "RShoulderRoll",
                                                      "RElbowYaw",
                                                      "RElbowRoll",
                                                      "RWristYaw" );
  AL::ALValue joints;
  AL::ALValue times;

  motionProxy_->setStiffnesses( names, 1.0 );

  joints.arraySetSize( 5 );
  times.arraySetSize( 5 );

  for (size_t j = 0; j < 5; ++j) {
    AL::ALValue angles;
    AL::ALValue ttr;

    angles.arraySetSize( traj_size );
    ttr.arraySetSize( traj_size );

    joints[j] = angles;
    times[j] = ttr;

    float time_to_reach_for_pt = 0.0;
    for (size_t jp = 0; jp < traj_size; ++jp) {
      joints[j][jp] = clamp( trajectory[jp][j], (isLeftArm ? L_SHOULDER_PITCH + j : R_SHOULDER_PITCH + j) );;
      time_to_reach_for_pt += times_to_reach[jp];
      times[j][jp] = time_to_reach_for_pt;
    }
  }
  try {
    if (inpost) {
      motionProxy_->post.angleInterpolation( names, joints, times, true );
    }
    else {
      motionProxy_->angleInterpolation( names, joints, times, true );
    }
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle interpolation to %s", joints.toString().c_str() );
    return false;
  }
  return true;
}

bool NaoProxyManager::moveLegWithJointPos( bool isLeft, const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_)
    return false;

  int pos_size = positions.size();
  if (pos_size != 6) {
    return false;
  }
  AL::ALValue names = isLeft ? "LLeg" : "RLeg";
  AL::ALValue angles;

  motionProxy_->setStiffnesses( names, 1.0 );

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], (isLeft ? L_HIP_YAW_PITCH + i : R_HIP_YAW_PITCH +i ) );
  }
  try {
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;
}

bool NaoProxyManager::moveBodyWithJointPos( const std::vector<float> & positions, float frac_speed )
{
  if (!motionProxy_)
    return false;

  int pos_size = positions.size();
  if (pos_size != kNAOJointNo) {
    return false;
  }
  //AL::ALValue names = "Body";
  AL::ALValue names;

  names.arraySetSize( kNAOJointNo );
  for (int i = 0; i < kNAOJointNo; i++) {
    names[i] = kNAOBodyJoints[i];
  }

  AL::ALValue angles;

  motionProxy_->setStiffnesses( names, 1.0 );

  angles.arraySetSize( pos_size );
  for (int i = 0; i < pos_size; ++i) {
    angles[i] = clamp( positions[i], i );
  }
  try {
    motionProxy_->setAngles( names, angles, frac_speed );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }
  return true;
}

bool NaoProxyManager::moveBodyWithRawTrajectoryData( std::vector<std::string> joint_names, std::vector< std::vector<AngleControlPoint> > & key_frames,
                                                 std::vector< std::vector<float> > & time_stamps, bool isBezier, bool inpost )
{
  // minimal check in this method. Use under you own risk!
  // TODO: this is a silly wrapper function. Should just do a simple cast.
  size_t joint_size = joint_names.size();
  if (joint_size != key_frames.size() || joint_size != time_stamps.size()) {
    ERROR_MSG( "Inconsistent trajectory data specification." );
    return false;
  }

  AL::ALValue keys;
  AL::ALValue times;

  keys.arraySetSize( joint_size );
  times.arraySetSize( joint_size );

  for (int i = 0; i < joint_size; i++) {
    int key_size = key_frames[i].size();
    int ts_size = time_stamps[i].size();

    keys[i].arraySetSize( key_size );
    for (int j = 0; j < key_size; ++j) {
      if (isBezier) {
        keys[i][j] = AL::ALValue::array( key_frames[i][j].angle, AL::ALValue::array( key_frames[i][j].bparam1.type,
            key_frames[i][j].bparam1.dtime, key_frames[i][j].bparam1.dangle ), AL::ALValue::array( key_frames[i][j].bparam2.type,
                key_frames[i][j].bparam2.dtime, key_frames[i][j].bparam2.dangle ) );
      }
      else {
        keys[i][j] = key_frames[i][j].angle;
      }
    }
    times[i].arraySetSize( ts_size );
    for (int j = 0; j < ts_size; ++j) {
      times[i][j] = time_stamps[i][j];
    }
  }

  motionProxy_->setStiffnesses( "Body", 1.0 );

  try {
    if (inpost)
      motionProxy_->post.angleInterpolationBezier( joint_names, times, keys );
    else
      motionProxy_->angleInterpolationBezier( joint_names, times, keys );
  }
  catch (...) {
    ERROR_MSG( "Unable to move joints in specified raw trajectories." );
    return false;
  }
  return true;
}

bool NaoProxyManager::setHandPosition( bool isLeft, float openRatio, bool keepStiff )
{
  float oratio = 1.0;
  if (openRatio <= 1.0 && openRatio >= 0.0) {
    oratio = openRatio;
  }

  AL::ALValue names;
  AL::ALValue angles;

  names.arraySetSize( 1 );
  angles.arraySetSize( 1 );

  if (isLeft) {
    names[0] = "LHand";
    angles[0] = oratio * ((float)jointLimits_[L_HAND][1] - (float)jointLimits_[L_HAND][0]);
    motionProxy_->setStiffnesses( "LHand", 1.0 );
  }
  else {
    names[0] = "RHand";
    angles[0] = oratio * ((float)jointLimits_[R_HAND][1] - (float)jointLimits_[R_HAND][0]);
    motionProxy_->setStiffnesses( "RHand", 1.0 );
  }

  try {
    motionProxy_->angleInterpolationWithSpeed( names, angles, 0.8 );
  }
  catch (...) {
    ERROR_MSG( "Unable to set angle to %s", angles.toString().c_str() );
    return false;
  }

  if (!keepStiff) {
    motionProxy_->setStiffnesses( (isLeft ? "LHand" : "RHand"), 0.0 );
  }
  return true;
}

int NaoProxyManager::loadAudioFile( const std::string & text )
{
  int audioID = -1;
  if (audioPlayerProxy_) {
    try {
      audioID = audioPlayerProxy_->loadFile( text );
    } catch (const std::exception &error) {
      ERROR_MSG( "Unable to load audio %s.\n", text.c_str() );
    }
  }
  return audioID;
}

void NaoProxyManager::unloadAudioFile( const int audioID )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadFile( audioID );
  }
}

void NaoProxyManager::unloadAllAudioFiles()
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.unloadAllFiles();
  }
}

void NaoProxyManager::playWebAudio( const std::string & url )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.playWebStream( url, 1.0, 0.0 );
  }
}

void NaoProxyManager::playAudioID( const int audioID, bool toBlock )
{
  if (audioPlayerProxy_) {
    if (toBlock) {
      audioPlayerProxy_->play( audioID );
    }
    else {
      audioPlayerProxy_->post.play( audioID );
    }
  }
}

int NaoProxyManager::getAudioVolume()
{
  if (audioDeviceProxy_) {
    return audioDeviceProxy_->getOutputVolume();
  }
  else {
    return 0.0;
  }
}

void NaoProxyManager::setAudioVolume( const int vol )
{
  if (vol < 0 || vol > 100 )
    return;

  if (audioDeviceProxy_) {
    audioDeviceProxy_->setOutputVolume( vol );
  }
}

void NaoProxyManager::pauseAudioID( const int audioID )
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.pause( audioID );
  }
}

void NaoProxyManager::stopAllAudio()
{
  if (audioPlayerProxy_) {
    audioPlayerProxy_->post.stopAll();
  }
}

bool NaoProxyManager::startBehaviour( const std::string & behaviour )
{
  if (!behaviourManagerProxy_)
    return false;

  try {
    behaviourManagerProxy_->startBehavior( behaviour );
  }
  catch (const ALError& e) {
    ERROR_MSG( "Unable to start behaviour %s.\n", behaviour.c_str() );
    return false;
  }
  return true;
}

bool NaoProxyManager::runBehaviour( const std::string & behaviour, bool inpost )
{
  if (!behaviourManagerProxy_)
    return false;

  try {
    if (inpost) {
      behaviourManagerProxy_->post.runBehavior( behaviour );
    }
    else {
      behaviourManagerProxy_->runBehavior( behaviour );
    }
  }
  catch (const ALError& e) {
    ERROR_MSG( "Unable to run behaviour %s.\n", behaviour.c_str() );
    return false;
  }

  return true;
}

void NaoProxyManager::stopBehaviour( const std::string & behaviour )
{
  if (behaviourManagerProxy_) {
    try {
      behaviourManagerProxy_->stopBehavior( behaviour );
    }
    catch (const ALError& e) {
      ERROR_MSG( "Unable to stop behaviour %s.\n", behaviour.c_str() );
    }
  }
}

void NaoProxyManager::stopAllBehaviours()
{
  if (behaviourManagerProxy_) {
    try {
      behaviourManagerProxy_->stopAllBehaviors();
    }
    catch (const ALError& e) {
      ERROR_MSG( "Unable to stop all behaviours.\n" );
    }
  }
}

std::vector<std::string> NaoProxyManager::getBehaviourList( bool installed )
{
  std::vector<std::string> list;
  if (behaviourManagerProxy_) {
    if (installed) {
      return behaviourManagerProxy_->getInstalledBehaviors();
    }
    else {
      return behaviourManagerProxy_->getLoadedBehaviors();
    }
  }
  return list;
}

void NaoProxyManager::setChestLED( const NAOLedColour colour )
{
  if (ledProxy_) {
    if (isChestLEDPulsating_) {
      isChestLEDPulsating_ = false;
      pthread_join( runningThread_, NULL );
    }
    switch (colour) {
      case WHITE:
        ledProxy_->on( "ChestLeds" );
        break;
      case BLANK:
        ledProxy_->off( "ChestLeds" );
        break;
      case RED:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case BLUE:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case GREEN:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->off( "ChestLedsRed" );
        break;
      case YELLOW:
        ledProxy_->off( "ChestLedsBlue" );
        ledProxy_->on( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      case PINK:
        ledProxy_->on( "ChestLedsBlue" );
        ledProxy_->off( "ChestLedsGreen" );
        ledProxy_->on( "ChestLedsRed" );
        break;
      default:
        break;
    }
  }
}

void NaoProxyManager::pulsatingChestLED( const NAOLedColour colour1, const NAOLedColour colour2, const float period )
{
  if (!ledProxy_)
    return;

  pthread_mutex_lock( &t_mutex_ );

  ledColourHex_.arraySetSize( 2 );
  ledColourHex_[0] = this->colour2Hex( colour1 );
  ledColourHex_[1] = this->colour2Hex( colour2 );
  ledChangePeriod_.arraySetSize( 2 );
  ledChangePeriod_[0] = ledChangePeriod_[1] = period;

  pthread_mutex_unlock( &t_mutex_ );

  if (isChestLEDPulsating_)
    return;

  isChestLEDPulsating_ = true;

  if (pthread_create( &runningThread_, NULL, pulse_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to pulse LED.\n" );
    return;
  }
}

void NaoProxyManager::getBatteryStatus( int & percentage, bool & isplugged, bool & ischarging, bool & isdischarging )
{
  if (memoryProxy_) {
    percentage = memoryProxy_->getData( "BatteryChargeChanged" );
    isplugged = memoryProxy_->getData( "BatteryPowerPluggedChanged" );
    ischarging = memoryProxy_->getData( "BatteryChargingFlagChanged" );
    isdischarging = memoryProxy_->getData( "BatteryDisChargingFlagChanged" );
  }
}

void NaoProxyManager::fini()
{
  if (isChestLEDPulsating_) {
    isChestLEDPulsating_ = false;
    pthread_join( runningThread_, NULL );
  }
  if (speechProxy_) {
    speechProxy_.reset();
  }
  if (motionProxy_) {
    motionProxy_.reset();
  }
  if (ledProxy_) {
    ledProxy_.reset();
  }
  if (audioDeviceProxy_) {
    audioDeviceProxy_.reset();
  }
  if (audioPlayerProxy_) {
    audioPlayerProxy_.reset();
  }
  if (behaviourManagerProxy_) {
    behaviourManagerProxy_.reset();
  }
}

// helper function
void NaoProxyManager::continuePulseChestLED()
{
  fd_set dummyFDSet;
  struct timeval timeout;

  FD_ZERO( &dummyFDSet );

  while (isChestLEDPulsating_) {
    pthread_mutex_lock( &t_mutex_ );
    ledProxy_->fadeListRGB( "ChestLeds", ledColourHex_, ledChangePeriod_ );
    pthread_mutex_unlock( &t_mutex_ );

    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;
    select( 1, &dummyFDSet, NULL, NULL, &timeout );
  }
}

void NaoProxyManager::timeoutCheck()
{
  fd_set dummyFDSet;
  struct timeval now, timeout;

  FD_ZERO( &dummyFDSet );

  do {
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;
    select( 1, &dummyFDSet, NULL, NULL, &timeout );
    gettimeofday( &now, NULL );
  } while (((now.tv_sec - cmdTimeStamp_.tv_sec) * 1000000 + now.tv_usec - cmdTimeStamp_.tv_usec) < kMotionCommandGapTolerance);
  this->cancelBodyMovement();
  timeoutThread_ = (pthread_t)NULL;
}

float NaoProxyManager::clamp( float val, int jointInd )
{
  if (jointInd >= jointLimits_.getSize()) {
    ERROR_MSG( "invalid joint index %d\n", jointInd );
    return val;
  }

  if (val < (float)jointLimits_[jointInd][0]) {
    return (float)jointLimits_[jointInd][0];
  }
  else if (val > (float)jointLimits_[jointInd][1]) {
    return (float)jointLimits_[jointInd][1];
  }
  else {
    return val;
  }
}

int NaoProxyManager::colour2Hex( const NAOLedColour colour )
{
  int retval = 0;
  switch (colour) {
    case WHITE:
      retval = 0x00ffffff;
      break;
    case RED:
      retval = 0x00ff0000;
      break;
    case BLUE:
      retval = 0x000000ff;
      break;
    case GREEN:
      retval = 0x0000ff00;
      break;
    case YELLOW:
      retval = 0x00ffff00;
      break;
    case PINK:
      retval = 0x00ff00ff;
      break;
    default:
      break;
  }
  return retval;
}
} // namespace pyride
