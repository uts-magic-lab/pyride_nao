/*
 *  PyNAOModule.cpp
 *  
 */

#include <pthread.h>
#include <string>
#include "PyNAOModule.h"
#include "NaoProxyManager.h"

namespace pyride {

#define PYRIDE_ROBOT_MODEL  "NAO"

PyDoc_STRVAR( PyNAO_doc, \
             "PyNAO is an extension module for PyRIDE on NAO robot." );

/*! \class PyNAO
 *  \brief PyNAO is the main Python extension module of PyRIDE on Aldebaran NAO robot.
 *
 *  PyNAO module consists of a set of callable Python methods specifically related to
 *  NAO low level functionalities and a set of callback functions that should be implemented
 *  by NAO programmers.
 */

PyNAOModule * PyNAOModule::s_pyNAOModule = NULL;

static const char *kLeftArmKWlist[] = { "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_elbow_yaw_joint", "l_elbow_roll_joint", "frac_max_speed", NULL };
static const char *kRightArmKWlist[] = { "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_elbow_yaw_joint", "r_elbow_roll_joint", "frac_max_speed", NULL };

static const char *kLeftLegKWlist[] = { "l_hip_yaw_pitch_joint", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_pitch_joint", "l_ankle_pitch_joint",
    "l_ankle_roll_joint", "frac_max_speed", NULL };
static const char *kRightLegKWlist[] = { "r_hip_yaw_pitch_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_pitch_joint", "r_ankle_pitch_joint",
    "r_ankle_roll_joint", "frac_max_speed", NULL };

static const char *kBodyKWlist[] = { "head_yaw_joint", "head_pitch_joint",
  "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_elbow_yaw_joint",
  "l_elbow_roll_joint", "l_hip_yaw_pitch_joint", "l_hip_roll_joint", "l_hip_pitch_joint",
  "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint"
  "r_hip_yaw_pitch_joint", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_pitch_joint",
  "r_ankle_pitch_joint", "r_ankle_roll_joint", "r_shoulder_pitch_joint",
  "r_shoulder_roll_joint", "r_elbow_yaw_joint", "r_elbow_roll_joint",
  "frac_max_speed", NULL };

// helper function

static bool colourStr2ID( const char * colourStr, NAOLedColour & colourID )
{
  if (!colourStr)
    return false;

  if (!strcasecmp( colourStr, "red" )) {
    colourID = RED;
  }
  else if (!strcasecmp( colourStr, "green" )) {
    colourID = GREEN;
  }
  else if (!strcasecmp( colourStr, "blue" )) {
    colourID = BLUE;
  }
  else if (!strcasecmp( colourStr, "white" )) {
    colourID = WHITE;
  }
  else if (!strcasecmp( colourStr, "yellow" )) {
    colourID = YELLOW;
  }
  else if (!strcasecmp( colourStr, "pink" )) {
    colourID = PINK;
  }
  else if (!strcasecmp( colourStr, "blank" )) {
    colourID = BLANK;
  }
  else {
    return false;
  }
  return true;
}

static PyObject * PyModule_write( PyObject *self, PyObject * args )
{
  char * msg;
  std::string outputMsg;

  if (!PyArg_ParseTuple( args, "s", &msg )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  // Next send it to all (active) clients.
  while (*msg) {
    if (*msg == '\n')
      outputMsg += "\r\n";
    else
      outputMsg += *msg;
    msg++;
  }
  
  PyNAOModule::instance()->write( outputMsg.c_str() );
  
  Py_RETURN_NONE;
}

static PyObject * PyModule_SetTeamMemberID( PyObject *self, PyObject * args )
{
  int teamID, teamColour;
  if (!PyArg_ParseTuple( args, "ii", &teamID, &teamColour )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (teamID < 0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setTeamMemberID: invalid "
                 "team member ID %d!", teamID );
    
    return NULL;
  }
  if (teamColour < BlueTeam || teamColour > PinkTeam) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setTeamMemberID: invalid "
                 "team colour %d! Valid format: %d = Pink; %d = Blue", teamColour, PinkTeam, BlueTeam );

    return NULL;
  }

  ServerDataProcessor::instance()->setTeamMemberID( teamID, (TeamColour)teamColour );
  PyNAOModule::instance()->clientID( ServerDataProcessor::instance()->clientID() );

  Py_RETURN_NONE;
}

static PyObject * PyModule_sendTeamMessage( PyObject *self, PyObject * args )
{
  char * dataStr = NULL;
  
  if (!PyArg_ParseTuple( args, "s", &dataStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  PyNAOModule::instance()->sendTeamMessage( dataStr );
  Py_RETURN_NONE;
}

static PyObject * PyModule_NaoSayWithVolume( PyObject * self, PyObject * args )
{
  float volume = 0.0;
  char * dataStr = NULL;
  PyObject * toBlockObj = NULL;
  
  if (!PyArg_ParseTuple( args, "s|fO", &dataStr, &volume, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (toBlockObj && !PyBool_Check( toBlockObj )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.say: third parameter must be a boolean!" );
    return NULL;
  }
  if (volume < 0.0 || volume > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.say: invalid voice volume!" );
    return NULL;
  }
  if (dataStr) {
    NaoProxyManager::instance()->sayWithVolume( string( dataStr ), volume,
                                               (toBlockObj && PyObject_IsTrue( toBlockObj )) );
  }
  Py_RETURN_NONE;
}

/*! \fn moveHeadTo(head_yaw, head_pitch,is_absolute)
 *  \memberof PyNAO
 *  \brief Move the NAO head to a specific yaw and pitch position. This new position could be either absolute head position or a relative position w.r.t the current head position.
 *  \param float head_yaw. Must be in radian.
 *  \param float head_pitch. Must be in radian.
 *  \param bool is_absolute. Optional. True = inputs represent absolute value for the new head position; False = inputs represent relative head position w.r.t. to the current head position.
 *  \return None.
 */
static PyObject * PyModule_NaoMoveHeadTo( PyObject * self, PyObject * args )
{
  float yaw = 0.0;
  float pitch = 0.0;
  bool absolute = false;
  PyObject * isYesObj = NULL;

  if (!PyArg_ParseTuple( args, "ff|O", &yaw, &pitch, &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      absolute = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.moveHeadTo: the third optional parameter must be a boolean!" );
      return NULL;
    }
  }

  NaoProxyManager::instance()->moveHeadTo( yaw, pitch, absolute );
  Py_RETURN_NONE;
}

static PyObject * PyModule_NaoUpdateHeadPos( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;

  if (!PyArg_ParseTuple( args, "dd", &yaw, &pitch )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  NaoProxyManager::instance()->updateHeadPos( yaw, pitch );
  Py_RETURN_NONE;
}

/*! \fn sit(is_relax)
 *  \memberof PyNAO
 *  \brief Move the robot into a sitting pose.
 *  \param bool is_relax. Optional. True = Relaxed sitting pose (stiffness is off); False = Non-relaxed sitting pose.
 *  \return None.
 */
static PyObject * PyModule_NaoSit( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;

  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.sit: the parameter must be a boolean!" );
      return NULL;
    }
  }
  NaoProxyManager::instance()->sit( isYes );
  Py_RETURN_NONE;
}

/*! \fn stand(is_init)
 *  \memberof PyNAO
 *  \brief Move the robot into a standing pose.
 *  \param bool is_init. Optional. True = A standing pose ready for walk; False = Default standing pose.
 *  \return None.
 */
static PyObject * PyModule_NaoStand( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;
  
  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.stand: the parameter must be a boolean!" );
      return NULL;
    }
  }
  NaoProxyManager::instance()->stand( isYes );
  Py_RETURN_NONE;
}

/*! \fn lyingDown(is_bellyup)
 *  \memberof PyNAO
 *  \brief Move the robot into a lying down pose.
 *  \param bool is_bellyup. Optional. True = belly up; False = belly down.
 *  \return None.
 */
static PyObject * PyModule_NaoLyingDown( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  bool isYes = false;
  
  if (!PyArg_ParseTuple( args, "|O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      isYes = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.lyingDown: the parameter must be a boolean!" );
      return NULL;
    }
  }
  NaoProxyManager::instance()->lyingDown( isYes );
  Py_RETURN_NONE;
}

/*! \fn crouch()
 *  \memberof PyNAO
 *  \brief Move the robot into a crouch pose.
 *  \return None.
 *  \note Stiffness will be turned off.
 */
static PyObject * PyModule_NaoCrouch( PyObject * self )
{
  NaoProxyManager::instance()->crouch();
  Py_RETURN_NONE;
}

/*! \fn getHeadPos()
 *  \memberof PyNAO
 *  \brief Get the current robot head yaw and pitch in radian.
 *  \return tuple(head_yaw, head_pitch)
 */
static PyObject * PyModule_NaoGetHeadPos( PyObject * self )
{
  float yaw = 0.0;
  float pitch = 0.0;
  if (NaoProxyManager::instance()->getHeadPos( yaw, pitch )) {
    return Py_BuildValue( "(ff)", yaw, pitch );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyNAO.getHeadPos: unable to get head position!" );
    return NULL;
  }
}
  
/*! \fn setHeadStiffness(stiffness)
 *  \memberof PyNAO
 *  \brief Set the stiffness of the NAO head.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_NaoSetHeadStiffness( PyObject * self, PyObject * args )
{
  float stiff = 0.0;
  
  if (!PyArg_ParseTuple( args, "f", &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setHeadStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }
  
  NaoProxyManager::instance()->setHeadStiffness( stiff );
  Py_RETURN_NONE;
}

/*! \fn setArmStiffness(left_arm,stiffness)
 *  \memberof PyNAO
 *  \brief Set the stiffness of a NAO' arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_NaoSetArmStiffness( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "Of", &isYesObj, &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( isYesObj )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setArmStiffness: the first parameter must be a boolean!" );
    return NULL;
  }
  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setArmStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  NaoProxyManager::instance()->setArmStiffness( PyObject_IsTrue( isYesObj ), stiff );
  Py_RETURN_NONE;
}

/*! \fn setLegStiffness(left_leg,stiffness)
 *  \memberof PyNAO
 *  \brief Set the stiffness of a NAO' leg.
 *  \param bool left_leg. True for left leg; False for right leg.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_NaoSetLegStiffness( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  float stiff = 0.0;
  
  if (!PyArg_ParseTuple( args, "Of", &isYesObj, &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( isYesObj )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setLegStiffness: the first parameter must be a boolean!" );
    return NULL;
  }
  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setLegStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }
  
  NaoProxyManager::instance()->setLegStiffness( PyObject_IsTrue( isYesObj ), stiff );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectory(joint_trajectory,is_blocking)
 *  \memberof PyNAO
 *  \brief Move a NAO arm to a sequence of joint positions, i.e. trajectory.
 *  \param list joint_trajectory. A list of joint position dictionaries with the same structure of the PyNAO.moveArmWithJointPos.
 *  \param bool is_blocking. Optional. True = blocking call; False = unblocking call.
 *  \return None.
 */
static PyObject * PyModule_NaoMoveArmWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  PyObject * isYesObj = NULL;
  bool inpost = false;

  if (!PyArg_ParseTuple( args, "O|O", &trajObj, &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  int listSize = 0;
  
  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }
  
  if (isYesObj) {
    if (PyBool_Check( isYesObj )) {
      inpost = PyObject_IsTrue( isYesObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: the second parameter must be a boolean!" );
      return NULL;
    }
  }
  
  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right
  
  std::vector< std::vector<float> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;
  
  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 4) {
      PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 4 joint entries for a NAO arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: input list contains"
                     " values not related to NAO arms!" );
        return NULL;
      }
    }
    
    std::vector<float> arm_joint_pos( 4, 0.0 );
    
    for (int k = 0; k < 4; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( arm_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }
  
  NaoProxyManager::instance()->moveArmWithJointTrajectory( (armsel == 1), trajectory,
                                                          times_to_reach, inpost );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyNAO
 *  \brief Move a NAO's arm to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of arm joint positions in radian.
 *  The dictionary must the same structure as the return of PyNAO.getArmJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
 */
static PyObject * PyModule_NaoMoveArmWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float s_p_j, s_r_j, e_y_j, e_r_j;
  float frac_max_speed = 0.5;
  
  bool isLeftArm = false;
  
  if (PyArg_ParseTupleAndKeywords( args, keywds, "ffff|f", (char**)kLeftArmKWlist,
                                  &s_p_j, &s_r_j, &e_y_j, &e_r_j, &frac_max_speed ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "ffff|f", (char**)kRightArmKWlist,
                                     &s_p_j, &s_r_j, &e_y_j, &e_r_j, &frac_max_speed ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }
  
  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.moveArmWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }
  
  std::vector<float> positions( 4, 0.0 );
  positions[0] = s_p_j;
  positions[1] = s_r_j;
  positions[2] = e_y_j;
  positions[3] = e_r_j;

  NaoProxyManager::instance()->moveArmWithJointPos( isLeftArm, positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn moveLegWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyNAO
 *  \brief Move a NAO's leg to the specified joint position with a certain speed.
 *  \param dict joint_position. A dictionary of leg joint positions in radian.
 *  The dictionary must the same structure as the return of PyNAO.getLegJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
 */
static PyObject * PyModule_NaoMoveLegWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float h_y_p_j, h_r_j, h_p_j, k_p_j, a_p_j, a_r_j;
  float frac_max_speed = 0.5;
  
  bool isLeftLeg = false;
  
  if (PyArg_ParseTupleAndKeywords( args, keywds, "ffffff|f", (char**)kLeftLegKWlist,
                                  &h_y_p_j, &h_r_j, &h_p_j, &k_p_j, &a_p_j, &a_r_j,
                                  &frac_max_speed ))
  {
    isLeftLeg = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "ffffff|f", (char**)kRightLegKWlist,
                                     &h_y_p_j, &h_r_j, &h_p_j, &k_p_j, &a_p_j, &a_r_j,
                                     &frac_max_speed ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }
  
  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.moveLegWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }
  
  std::vector<float> positions( 6, 0.0 );
  positions[0] = h_y_p_j;
  positions[1] = h_r_j;
  positions[2] = h_p_j;
  positions[3] = k_p_j;
  positions[4] = a_p_j;
  positions[5] = a_r_j;
  
  NaoProxyManager::instance()->moveLegWithJointPos( isLeftLeg, positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn moveBodyWithJointPos(joint_position, frac_max_speed)
 *  \memberof PyNAO
 *  \brief Move the NAO body joints to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of body joint positions in radian.
 *  The dictionary must the same structure as the return of PyNAO.getBodyJointPositions.
 *  \param float frac_max_speed. Fraction of the maximum motor speed.
 *  \return None.
 */
static PyObject * PyModule_NaoMoveBodyWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  float l_h_y_p_j, l_h_r_j, l_h_p_j, l_k_p_j, l_a_p_j, l_a_r_j;
  float r_h_y_p_j, r_h_r_j, r_h_p_j, r_k_p_j, r_a_p_j, r_a_r_j;
  float l_s_p_j, l_s_r_j, l_e_y_j, l_e_r_j;
  float r_s_p_j, r_s_r_j, r_e_y_j, r_e_r_j;
  float h_y_j, h_p_j;

  float frac_max_speed = 0.5;
  
  if (!PyArg_ParseTupleAndKeywords( args, keywds, "ffffffffffffffffffffff|f",
                                   (char**)kBodyKWlist, &h_y_j, &h_p_j,
                                   &l_s_p_j, &l_s_r_j, &l_e_y_j, &l_e_r_j,
                                   &l_h_y_p_j, &l_h_r_j, &l_h_p_j, &l_k_p_j, &l_a_p_j, &l_a_r_j,
                                   &r_h_y_p_j, &r_h_r_j, &r_h_p_j, &r_k_p_j, &r_a_p_j, &r_a_r_j,
                                   &r_s_p_j, &r_s_r_j, &r_e_y_j, &r_e_r_j,
                                   &frac_max_speed ))
  {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (frac_max_speed > 1.0 || frac_max_speed < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.moveBodyWithJointPos: fraction of max speed must be a value within [0.0, 1.0]!" );
    return NULL;
  }
  
  std::vector<float> positions( 22, 0.0 );
  positions[0] = h_y_j;
  positions[1] = h_p_j;
  
  positions[2] = l_s_p_j;
  positions[3] = l_s_r_j;
  positions[4] = l_e_y_j;
  positions[5] = l_e_r_j;

  positions[6] = l_h_y_p_j;
  positions[7] = l_h_r_j;
  positions[8] = l_h_p_j;
  positions[9] = l_k_p_j;
  positions[10] = l_a_p_j;
  positions[11] = l_a_r_j;

  positions[12] = r_h_y_p_j;
  positions[13] = r_h_r_j;
  positions[14] = r_h_p_j;
  positions[15] = r_k_p_j;
  positions[16] = r_a_p_j;
  positions[17] = r_a_r_j;
  
  positions[18] = r_s_p_j;
  positions[19] = r_s_r_j;
  positions[20] = r_e_y_j;
  positions[21] = r_e_r_j;

  NaoProxyManager::instance()->moveBodyWithJointPos( positions, frac_max_speed );
  Py_RETURN_NONE;
}

/*! \fn getArmJointPositions(left_arm)
 *  \memberof PyNAO
 *  \brief Get the current joint positions of one of the NAO arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \return dictionary(arm_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_NaoGetArmJointPositions( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  PyObject * usbObj = NULL;

  bool useSensor = false;
  
  if (!PyArg_ParseTuple( args, "O|O", &armsel, &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.getArmJointPositions: first input parameter must be a boolean!" );
    return NULL;
  }

  bool isLeftArm = PyObject_IsTrue( armsel );

  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.getArmJointPositions: second input parameter must be a boolean!" );
      return NULL;
    }
  }

  std::vector<float> positions( 4, 0.0 );

  NaoProxyManager::instance()->getArmJointsPos( isLeftArm, positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 4; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, (isLeftArm ? kLeftArmKWlist[i] : kRightArmKWlist[i]), numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

/*! \fn getLegJointPositions(left_leg)
 *  \memberof PyNAO
 *  \brief Get the current joint positions of one of the NAO left.
 *  \param bool left_leg. True for left leg; False for right leg.
 *  \return dictionary(leg_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_NaoGetLegJointPositions( PyObject * self, PyObject * args )
{
  PyObject * legsel = NULL;
  PyObject * usbObj = NULL;
  
  bool useSensor = false;
  
  if (!PyArg_ParseTuple( args, "O|O", &legsel, &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( legsel )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.getLegJointPositions: first input parameter must be a boolean!" );
    return NULL;
  }
  
  bool isLeftLeg = PyObject_IsTrue( legsel );
  
  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.getLegJointPositions: second input parameter must be a boolean!" );
      return NULL;
    }
  }
  
  std::vector<float> positions( 6, 0.0 );
  
  NaoProxyManager::instance()->getLegJointsPos( isLeftLeg, positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 6; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, (isLeftLeg ? kLeftLegKWlist[i] : kRightLegKWlist[i]), numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

/*! \fn getBodyJointPositions(use_sensor_data)
 *  \memberof PyNAO
 *  \brief Get the current joint positions of NAO body joints.
 *  \param bool use_sensor_data. True = using sensor reading; False = not using sensor.
 *  \return dictionary(body_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_NaoGetBodyJointPositions( PyObject * self, PyObject * args )
{
  PyObject * usbObj = NULL;
  
  bool useSensor = false;
  
  if (!PyArg_ParseTuple( args, "|O", &usbObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (usbObj) {
    if (PyBool_Check( usbObj )) {
      useSensor = PyObject_IsTrue( usbObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyNAO.getBodyJointPositions: the optional input parameter must be a boolean!" );
      return NULL;
    }
  }
  
  std::vector<float> positions( 22, 0.0 );
  
  NaoProxyManager::instance()->getBodyJointsPos( positions, useSensor );
  PyObject * retObj = PyDict_New();
  for (int i = 0; i < 4; i++) {
    PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
    PyDict_SetItemString( retObj, kBodyKWlist[i], numObj );
    Py_DECREF( numObj );
  }
  return retObj;
}

/** @name Audio Management Functions
 *
 */
/**@{*/
/*! \fn loadAudioFile(file_name)
 *  \memberof PyNAO
 *  \brief Load an audio file into the robot system.
 *  \param str file_name. Audio file path. Must be a full path.
 *  \return int audio ID.
 */
static PyObject * PyModule_NaoLoadAudioFile( PyObject * self, PyObject * args )
{
  char * text = NULL;
  int audioID = -1;
  
  if (!PyArg_ParseTuple( args, "s", &text )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (text) {
    audioID = NaoProxyManager::instance()->loadAudioFile( text );
  }
  if (audioID == -1) {
    PyErr_Format( PyExc_ValueError, "PyNAO.loadAudioFile: unable to load audio file %s!", text );
    return NULL;
  }
  return Py_BuildValue( "i", audioID );
}

/*! \fn unloadAudioFile(audio_id)
 *  \memberof PyNAO
 *  \brief Unload an audio from the robot system (free memory).
 *  \param int audio_id. Loaded audio file ID.
 *  \return None.
 */
static PyObject * PyModule_NaoUnloadAudioFile( PyObject * self, PyObject * args )
{
  int audioID = 0;
  
  if (!PyArg_ParseTuple( args, "i", &audioID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (audioID > 0) {
    NaoProxyManager::instance()->unloadAudioFile( audioID );
  }
  Py_RETURN_NONE;
}

/*! \fn unloadAllAudioFile()
 *  \memberof PyNAO
 *  \brief Unload all audio from the robot system.
 *  \return None.
 */
static PyObject * PyModule_NaoUnloadAllAudioFiles( PyObject * self )
{
  NaoProxyManager::instance()->unloadAllAudioFiles();
  Py_RETURN_NONE;
}

static PyObject * PyModule_NaoPlayWebAudio( PyObject * self, PyObject * args )
{
  char * text = NULL;
  
  if (!PyArg_ParseTuple( args, "s", &text )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (text) {
    NaoProxyManager::instance()->playWebAudio( text );
  }
  Py_RETURN_NONE;
}

/*! \fn playAudioID(audio_id)
 *  \memberof PyNAO
 *  \brief Play a loaded audio file.
 *  \param int audio_id. Loaded audio file ID.
 *  \return None.
 */
static PyObject * PyModule_NaoPlayAudioID( PyObject * self, PyObject * args )
{
  int audioID = 0;
  PyObject * toBlockObj = NULL;

  if (!PyArg_ParseTuple( args, "i|O", &audioID, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (toBlockObj && !PyBool_Check( toBlockObj )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.say: second parameter should be a boolean!" );
    return NULL;
  }
  if (audioID > 0) {
    NaoProxyManager::instance()->playAudioID( audioID, (toBlockObj && PyObject_IsTrue( toBlockObj )) );
  }
  Py_RETURN_NONE;
}
  
/*! \fn getAudioVolume()
 *  \memberof PyNAO
 *  \brief Get the current master audio volume.
 *  \return int audio volume between [0,100].
 */
static PyObject * PyModule_NaoGetAudioVolume( PyObject * self )
{
  int volume = NaoProxyManager::instance()->getAudioVolume();
  return Py_BuildValue( "i", volume );
}

/*! \fn setAudioVolume(volume)
 *  \memberof PyNAO
 *  \brief Set the master audio volume.
 *  \param int volume. Must be between [0,100]
 *  \return None.
 */
static PyObject * PyModule_NaoSetAudioVolume( PyObject * self, PyObject * args )
{
  int volume = 50;
  
  if (!PyArg_ParseTuple( args, "i", &volume )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (volume < 0 || volume > 100) {
    PyErr_Format( PyExc_ValueError, "PyNAO.setAudioVolume: invalid audio volume, must be within [0..100]!" );
    return NULL;
  }
  
  NaoProxyManager::instance()->setAudioVolume( volume );
  Py_RETURN_NONE;
}

/*! \fn pauseAudioID(audio_id)
 *  \memberof PyNAO
 *  \brief Pause a playing audio.
 *  \param int audio_id. Audo file ID.
 *  \return None.
 */
static PyObject * PyModule_NaoPauseAudioID( PyObject * self, PyObject * args )
{
  int audioID = 0;
  
  if (!PyArg_ParseTuple( args, "i", &audioID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (audioID > 0) {
    NaoProxyManager::instance()->pauseAudioID( audioID );
  }
  Py_RETURN_NONE;
}

/*! \fn stopAllAudio()
 *  \memberof PyNAO
 *  \brief Stop playing all audio files.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_NaoStopAllAudio( PyObject * self )
{
  NaoProxyManager::instance()->stopAllAudio();
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn setChestLED(colour)
 *  \memberof PyNAO
 *  \brief Set NAO's chest LED to a colour.
 *  \param str colour. Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 *  \return None.
 */
static PyObject * PyModule_NaoSSetChestLED( PyObject * self, PyObject * args )
{
  char * colourStr = NULL;
  NAOLedColour colourID;
  
  if (!PyArg_ParseTuple( args, "s", &colourStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (colourStr2ID( colourStr, colourID )) {
    NaoProxyManager::instance()->setChestLED( colourID );
  }
  else {
    PyErr_Format( PyExc_ValueError, "PyNAO.setChestLED: invalid input colour."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }
  Py_RETURN_NONE;
}

/*! \fn pluseChestLED(colour_one, colour_two, period)
 *  \memberof PyNAO
 *  \brief Periodically switch NAO's chest LED between the two input colours.
 *  \param str colour_one. Colour label one.
 *  \param str colour_one. Colour label two.
 *  \param int period. Time (in seconds) before switching LED colour.
 *  \return None.
 *  \note Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 */
static PyObject * PyModule_NaoPluseChestLED( PyObject * self, PyObject * args )
{
  char * colourStr1 = NULL;
  char * colourStr2 = NULL;

  float period = 0.5;

  NAOLedColour colourID1, colourID2;
  
  if (!PyArg_ParseTuple( args, "ss|f", &colourStr1, &colourStr2, &period )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!colourStr2ID( colourStr1, colourID1 ) || !colourStr2ID( colourStr2, colourID2 )) {
    PyErr_Format( PyExc_ValueError, "PyNAO.pluseChestLED: invalid input colour(s)."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }
  
  if (period <= 0.0) {
    PyErr_Format( PyExc_ValueError, "PyNAO.pluseChestLED: invalid pluse period." );
    return NULL;
  }
  
  NaoProxyManager::instance()->pulsatingChestLED( colourID1, colourID2, period );
  Py_RETURN_NONE;
}

/*! \fn getBatteryStatus()
 *  \memberof PyNAO
 *  \brief Return the current robot battery status.
 *  \return tuple(battery percentage, is_plugged_in, is_(dis)charging).
 */
/**@}*/
static PyObject * PyModule_NaoGetBatteryStatus( PyObject * self )
{
  int batpercent = 0;
  bool isplugged = false;
  bool ischarging = false;
  bool isdischarging = false;

  NaoProxyManager::instance()->getBatteryStatus( batpercent, isplugged, ischarging, isdischarging );

  if (ischarging) {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "charging" );
  }
  else if (isdischarging) {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "discharging" );
    
  }
  else {
    return Py_BuildValue( "(iss)", batpercent, isplugged ? "plugged in" :
                         "unplugged", "not charging" );
  }
}

#define INCLUDE_COMMON_PYMODULE_MEHTODS
#include "PyModulePyCommon.cpp"

static PyMethodDef PyModule_methods[] = {
  { "write", (PyCFunction)PyModule_write, METH_VARARGS,
    "standard output for UTS NAO Python console." },
  { "setTeamMemberID", (PyCFunction)PyModule_SetTeamMemberID, METH_VARARGS,
    "Set Nao team member ID and team colour." },
  { "sendTeamMessage", (PyCFunction)PyModule_sendTeamMessage, METH_VARARGS,
    "Send a message to the rest team members." },
  { "say", (PyCFunction)PyModule_NaoSayWithVolume, METH_VARARGS,
    "Let NAO speak with an optional volume." },
  { "moveHeadTo", (PyCFunction)PyModule_NaoMoveHeadTo, METH_VARARGS,
    "Move NAO head to a new position." },
  { "updateHeadPos", (PyCFunction)PyModule_NaoUpdateHeadPos, METH_VARARGS,
    "Change NAO head position with a specific angle in radian." },
  { "getHeadPos", (PyCFunction)PyModule_NaoGetHeadPos, METH_NOARGS,
    "Get NAO's head position." },
  { "setHeadStiffness", (PyCFunction)PyModule_NaoSetHeadStiffness, METH_VARARGS,
    "Set the stiffness of the NAO's head. " },
  { "sit", (PyCFunction)PyModule_NaoSit, METH_VARARGS,
    "Get NAO to sit in standard or relax mode (set optional input to True). " },
  { "stand", (PyCFunction)PyModule_NaoStand, METH_VARARGS,
    "Get NAO to stand up in standard or ready to walk mode (set optional input to True). " },
  { "crouch", (PyCFunction)PyModule_NaoCrouch, METH_NOARGS,
    "Get NAO to crouch. " },
  { "lyingDown", (PyCFunction)PyModule_NaoLyingDown, METH_VARARGS,
    "Get NAO to lying down either belly down or up. (set optional input to True). " },
  { "setArmStiffness", (PyCFunction)PyModule_NaoSetArmStiffness, METH_VARARGS,
    "Set the stiffness of the one of NAO's arms. " },
  { "setLegStiffness", (PyCFunction)PyModule_NaoSetLegStiffness, METH_VARARGS,
     "Set the stiffness of the one of NAO's legs. " },
  { "moveArmWithJointPos", (PyCFunction)PyModule_NaoMoveArmWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of NAO arms with specific joint positions." },
  { "moveArmWithJointTrajectory", (PyCFunction)PyModule_NaoMoveArmWithJointTraj, METH_VARARGS,
    "Move one of NAO arms with specific joint trajectory (a list of joint positions)." },
  { "moveLegWithJointPos", (PyCFunction)PyModule_NaoMoveLegWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of NAO legs with specific joint positions." },
  { "moveBodyWithJointPos", (PyCFunction)PyModule_NaoMoveBodyWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move NAO all body joint positions." },
  { "getArmJointPositions", (PyCFunction)PyModule_NaoGetArmJointPositions, METH_VARARGS,
    "Get joint positions of Nao's arms." },
  { "getLegJointPositions", (PyCFunction)PyModule_NaoGetLegJointPositions, METH_VARARGS,
    "Get joint positions of Nao's legs." },
  { "getBodyJointPositions", (PyCFunction)PyModule_NaoGetBodyJointPositions, METH_VARARGS,
    "Get full joint positions of Nao." },
  { "loadAudioFile", (PyCFunction)PyModule_NaoLoadAudioFile, METH_VARARGS,
    "Load an audio file on NAO." },
  { "unloadAudioFile", (PyCFunction)PyModule_NaoUnloadAudioFile, METH_VARARGS,
    "Unload an audio file from NAO." },
  { "unloadAllAudioFiles", (PyCFunction)PyModule_NaoUnloadAllAudioFiles, METH_NOARGS,
    "Unload all audio files from NAO." },
  { "playWebAudio", (PyCFunction)PyModule_NaoPlayWebAudio, METH_VARARGS,
    "Play a web audio stream on NAO." },
  { "playAudioID", (PyCFunction)PyModule_NaoPlayAudioID, METH_VARARGS,
    "Play an audio file on NAO." },
  { "getAudioVolume", (PyCFunction)PyModule_NaoGetAudioVolume, METH_NOARGS,
    "Get master audio volume on NAO." },
  { "setAudioVolume", (PyCFunction)PyModule_NaoSetAudioVolume, METH_VARARGS,
    "Set master audio volume on NAO." },
  { "pauseAudioID", (PyCFunction)PyModule_NaoPauseAudioID, METH_VARARGS,
    "Pause a playing audio file." },
  { "stopAllAudio", (PyCFunction)PyModule_NaoStopAllAudio, METH_NOARGS,
    "Stop playing all audio on NAO." },
  { "setChestLED", (PyCFunction)PyModule_NaoSSetChestLED, METH_VARARGS,
    "Set the colour of the chest LEDs on NAO." },
  { "pluseChestLED", (PyCFunction)PyModule_NaoPluseChestLED, METH_VARARGS,
    "Pluse the chest LED of NAO between two colours." },
  { "getBatteryStatus", (PyCFunction)PyModule_NaoGetBatteryStatus, METH_NOARGS,
    "Get the current battery status." },
#define DEFINE_COMMON_PYMODULE_METHODS
#include "PyModulePyCommon.cpp"
  { NULL, NULL, 0, NULL }           /* sentinel */
};

//#pragma mark PyNAOModule implmentation
PyNAOModule::PyNAOModule() : PyModuleExtension( "PyNAO" )
{

}

PyObject * PyNAOModule::createPyModule()
{
  return Py_InitModule3( "PyNAO", PyModule_methods, PyNAO_doc );
}

PyNAOModule * PyNAOModule::instance()
{
  if (!s_pyNAOModule)
    s_pyNAOModule = new PyNAOModule();
  
  return s_pyNAOModule;
}
} // namespace pyride
