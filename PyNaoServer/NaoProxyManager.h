/*
 *  NaoProxyManager.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 16/08/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef NaoProxyManager_h_DEFINED
#define NaoProxyManager_h_DEFINED
#include <string>
#include <pthread.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <alerror/alerror.h>
#include <alcommon/albroker.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alaudioplayerproxy.h>
#include <alproxies/alaudiodeviceproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/albehaviormanagerproxy.h>

namespace pyride {

using namespace AL;

typedef enum {
  WHITE = 0,
  BLANK,
  RED,
  BLUE,
  GREEN,
  YELLOW,
  PINK
} NAOLedColour;

enum { // standard configuration for NAO V4/V5.
  HEAD_YAW = 0,
  HEAD_PITCH,
  L_SHOULDER_PITCH,
  L_SHOULDER_ROLL,
  L_ELBOW_YAW,
  L_ELBOW_ROLL,
  L_WRIST_YAW,
  L_HIP_YAW_PITCH,
  L_HIP_ROLL,
  L_HIP_PITCH,
  L_KNEE_PITCH,
  L_ANKLE_PITCH,
  L_ANKLE_ROLL,
  R_HIP_YAW_PITCH,
  R_HIP_ROLL,
  R_HIP_PITCH,
  R_KNEE_PITCH,
  R_ANKLE_PITCH,
  R_ANKLE_ROLL,
  R_SHOULDER_PITCH,
  R_SHOULDER_ROLL,
  R_ELBOW_YAW,
  R_ELBOW_ROLL,
  R_WRIST_YAW,
  L_HAND,
  R_HAND
};

struct BezierParam
{
  int type;
  float dtime;
  float dangle;
  BezierParam() { type = -1; dtime = dangle = 0.0; };
};

typedef struct
{
  float angle;
  struct BezierParam bparam1;
  struct BezierParam bparam2;
} AngleControlPoint;

class NaoProxyManager
{
public:
  static NaoProxyManager * instance();
  ~NaoProxyManager();

  void initWithBroker( boost::shared_ptr<ALBroker> broker, boost::shared_ptr<ALMemoryProxy> memoryProxy );
  bool sayWithVolume( const std::string & text, float volume  = 0.0, bool toBlock = false );

  int loadAudioFile( const std::string & text );
  void unloadAudioFile( const int audioID );
  void unloadAllAudioFiles();
  void playWebAudio( const std::string & url );
  bool playAudioID( const int audioID );
  int  getAudioVolume();
  void setAudioVolume( const int vol );
  void pauseAudioID( const int audioID );
  void stopAllAudio();

  void setChestLED( const NAOLedColour colour );
  void pulsatingChestLED( const NAOLedColour colour1, const NAOLedColour colour2, const float period = 0.5 );
  void continuePulseChestLED();

  void getBatteryStatus( int & percentage, bool & isplugged, bool & ischarging, bool & isdischarging );

  bool getHeadPos( float & yaw, float & pitch );
  bool moveHeadTo( const float yaw, const float pitch, bool relative = false, float frac_speed = 0.05 );
  void updateHeadPos( const float yaw, const float pitch, const float speed = 0.1 );

  void getBodyJointsPos( std::vector<float> & positions,
                        bool useSensor = false );
  void getArmJointsPos( bool isLeft, std::vector<float> & positions,
                       bool useSensor = false );
  void getLegJointsPos( bool isLeft, std::vector<float> & positions,
                       bool useSensor = false );

  void setArmStiffness( bool isLeft, const float stiff );
  void setHeadStiffness( const float stiff );
  void setBodyStiffness( const float stiff );
  void setLegStiffness( bool isLeft, const float stiff );

  bool moveArmWithJointPos( bool isLeft, const std::vector<float> & positions,
                            float frac_speed = 0.5 );

  bool moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<float> > & trajectory,
                                                   std::vector<float> & times_to_reach );

  bool moveLegWithJointPos( bool isLeft, const std::vector<float> & positions,
                           float frac_speed = 0.5 );

  bool moveBodyWithJointPos( const std::vector<float> & positions,
                            float frac_speed = 0.5 );

  bool moveBodyWithRawTrajectoryData( std::vector<std::string> joint_names, std::vector< std::vector<AngleControlPoint> > & key_frames,
                                                   std::vector< std::vector<float> > & time_stamps, bool isBezier );

  bool setHandPosition( bool isLeft, float openRatio, bool keepStiff );

  void sit( bool relax = false );
  void stand( bool init = false );
  void crouch();
  void lyingDown( bool bellyUp = true );

  bool moveBodyTo( const RobotPose & pose, bool cancelPreviousMove = false );

  void updateBodyPose( const RobotPose & pose );

  bool startBehaviour( const std::string & behaviour );
  bool runBehaviour( const std::string & behaviour );
  void stopBehaviour( const std::string & behavour );
  void stopAllBehaviours();

  std::vector<std::string> getBehaviourList( bool installed = false );

  void timeoutCheck();

  void cancelBodyMovement();

  void fini();

  void blockedSpeech( const std::string & text, float volume );
  void blockedHeadMove( const float yaw, const float pitch, bool relative, float frac_speed );
  void blockedArmMove( bool isLeftArm, const std::vector<float> & positions, float frac_speed );
  void blockedArmMoveTraj( bool isLeftArm, std::vector< std::vector<float> > & trajectory,
      std::vector<float> & times_to_reach );
  void blockedBodyMoveWithData( std::vector<std::string> joint_names, std::vector< std::vector<AngleControlPoint> > & key_frames,
                                                   std::vector< std::vector<float> > & time_stamps, bool isBezier );
  void blockedBodyMoveTo( const RobotPose & pose );

  void blockedHandMove( bool isLeft, float openRatio, bool keepStiff );

  void blockedBehaviourRun( const std::string & behaviour );

  void blockedPlayAudio( const int audioID );

private:
  static NaoProxyManager * s_pNaoProxyManager;

  boost::shared_ptr<ALTextToSpeechProxy> speechProxy_;
  boost::shared_ptr<ALMotionProxy> motionProxy_;
  boost::shared_ptr<ALRobotPostureProxy> postureProxy_;
  boost::shared_ptr<ALLedsProxy> ledProxy_;
  boost::shared_ptr<ALAudioPlayerProxy> audioPlayerProxy_;
  boost::shared_ptr<ALAudioDeviceProxy> audioDeviceProxy_;
  boost::shared_ptr<ALMemoryProxy> memoryProxy_;
  boost::shared_ptr<ALBehaviorManagerProxy> behaviourManagerProxy_;

  boost::thread * speechThread_;
  boost::thread * headmoveThread_;
  boost::thread * larmmoveThread_;
  boost::thread * rarmmoveThread_;
  boost::thread * lhandmoveThread_;
  boost::thread * rhandmoveThread_;
  boost::thread * bodymoveThread_;
  boost::thread * behaviourThread_;
  boost::thread * audioThread_;

  //motion related data
  ALValue jointLimits_;

  struct timeval cmdTimeStamp_;

  bool moveInitialised_;
  bool isChestLEDPulsating_;
  bool speechCtrl_;
  bool headCtrl_;
  bool lArmCtrl_;
  bool rArmCtrl_;
  bool lHandCtrl_;
  bool rHandCtrl_;
  bool bodyCtrl_;
  bool behaviourCtrl_;
  bool audioCtrl_;

  ALValue ledColourHex_;
  ALValue ledChangePeriod_;
  pthread_t runningThread_;
  pthread_t timeoutThread_;
  pthread_mutex_t t_mutex_;
  pthread_mutexattr_t t_mta;

  NaoProxyManager();

  float clamp( float val, int jointInd );
  int colour2Hex( const NAOLedColour colour );

};
} // namespace pyride
#endif // NaoProxyManager_h_DEFINED
