/*
 *  PyNAOModule.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 18/06/10.
 *  Copyright 2010 Galaxy Network. All rights reserved.
 *
 */

#ifndef PY_NAO_MODULE_H
#define PY_NAO_MODULE_H

#include <PyModuleStub.h>

namespace pyride {

class PyNAOModule : public PyModuleExtension
{
public:
  static PyNAOModule * instance();
  
private:
  static PyNAOModule * s_pyNAOModule;

  PyNAOModule();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_NAO_MODULE_H
