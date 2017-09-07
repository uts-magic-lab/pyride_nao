/*
 *  AudioFeedbackStream.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 23/06/2017
 *
 */

#ifndef AUDIO_FEEDBACK_STREAM_H
#define AUDIO_FEEDBACK_STREAM_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <alcommon/alproxy.h>

#include <celt.h>

#include "RTPDataReceiver.h"
#include "PyModuleStub.h"

namespace pyride {

using namespace pyride_remote;
using namespace AL;

class AudioFeedbackStream
{
public:
  static AudioFeedbackStream * instance();
  ~AudioFeedbackStream();

  void initWithModule( boost::shared_ptr<ALProxy> audioDevice, PyModuleExtension * extension = NULL );

  void addClient();
  void removeClient();

private:
  int clientNo_;
  bool isRunning_;

  boost::shared_ptr<ALProxy> audioDevice_;

  RTPDataReceiver * dataStream_;
  PyModuleExtension * pyExtension_;

  CELTMode * celtMode_;
  CELTDecoder * audioDecoder_;

  boost::thread * streaming_data_thread_;

  static AudioFeedbackStream * s_pAudioFeedbackStream;

  AudioFeedbackStream();

  bool start();
  void stop();

  void grabAndDispatchAudioStreamData();
};

} // namespace pyride

#endif /* AUDIO_FEEDBACK_STREAM_H */
