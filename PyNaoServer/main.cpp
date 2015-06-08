#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "PyRideCommon.h"
#include "PyNaoServer.h"

#ifdef _WIN32
# define ALCALL __declspec(dllexport)
#else
# define ALCALL
#endif

using namespace std;
using namespace AL;
using namespace pyride;

#ifdef __cplusplus
extern "C"
{
#endif
  ALCALL int _createModule( boost::shared_ptr<ALBroker> pBroker )
  {      
    // init broker with the main broker inctance 
    // from the parent executable
    ALBrokerManager::setInstance( pBroker->fBrokerManager.lock() );
    ALBrokerManager::getInstance()->addBroker( pBroker );

    // create modules instance
    ALModule::createModule<PyNaoServer>( pBroker, "PyNaoServer" );
    return 0;
  }
  
  ALCALL int _closeModule(  )
  {
    // Delete module instance
    return 0;
  }
  
# ifdef __cplusplus
}
# endif
