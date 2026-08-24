#include "UnderwaterDronePhysics_FMU.h"

// include fmu header files, typedefs and macros
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "openmodelica.h"
#include "openmodelica_func.h"
#include "util/omc_error.h"
#include "UnderwaterDronePhysics_functions.h"

#include "simulation/solver/events.h"

// Set values for all variables that define a start value
OMC_DISABLE_OPT
void setDefaultStartValues(ModelInstance *comp) {
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[0].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[1].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[2].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[3].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[4].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[5].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[6].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[7].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[8].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[9].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[10].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[11].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[12].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[13].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[14].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[15].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[16].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[17].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realVarsData[18].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[19].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[20].attribute.start);
  put_real_element(0, 0, &comp->fmuData->modelData->realVarsData[21].attribute.start);
  put_real_element(5.0, 0, &comp->fmuData->modelData->realParameterData[0].attribute.start);
  put_real_element(15.0, 0, &comp->fmuData->modelData->realParameterData[1].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realParameterData[2].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realParameterData[3].attribute.start);
  put_real_element(0.0, 0, &comp->fmuData->modelData->realParameterData[4].attribute.start);
}
// Set values for all variables that define a start value
OMC_DISABLE_OPT
void setStartValues(ModelInstance *comp) {
  put_real_element(comp->fmuData->localData[0]->realVars[0], 0, &comp->fmuData->modelData->realVarsData[0].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[1], 0, &comp->fmuData->modelData->realVarsData[1].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[2], 0, &comp->fmuData->modelData->realVarsData[2].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[3], 0, &comp->fmuData->modelData->realVarsData[3].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[4], 0, &comp->fmuData->modelData->realVarsData[4].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[5], 0, &comp->fmuData->modelData->realVarsData[5].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[6], 0, &comp->fmuData->modelData->realVarsData[6].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[7], 0, &comp->fmuData->modelData->realVarsData[7].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[8], 0, &comp->fmuData->modelData->realVarsData[8].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[9], 0, &comp->fmuData->modelData->realVarsData[9].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[10], 0, &comp->fmuData->modelData->realVarsData[10].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[11], 0, &comp->fmuData->modelData->realVarsData[11].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[12], 0, &comp->fmuData->modelData->realVarsData[12].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[13], 0, &comp->fmuData->modelData->realVarsData[13].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[14], 0, &comp->fmuData->modelData->realVarsData[14].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[15], 0, &comp->fmuData->modelData->realVarsData[15].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[16], 0, &comp->fmuData->modelData->realVarsData[16].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[17], 0, &comp->fmuData->modelData->realVarsData[17].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[18], 0, &comp->fmuData->modelData->realVarsData[18].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[19], 0, &comp->fmuData->modelData->realVarsData[19].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[20], 0, &comp->fmuData->modelData->realVarsData[20].attribute.start);
  put_real_element(comp->fmuData->localData[0]->realVars[21], 0, &comp->fmuData->modelData->realVarsData[21].attribute.start);
  put_real_element(comp->fmuData->simulationInfo->realParameter[0], 0, &comp->fmuData->modelData->realParameterData[0].attribute.start);
  put_real_element(comp->fmuData->simulationInfo->realParameter[1], 0, &comp->fmuData->modelData->realParameterData[1].attribute.start);
  put_real_element(comp->fmuData->simulationInfo->realParameter[2], 0, &comp->fmuData->modelData->realParameterData[2].attribute.start);
  put_real_element(comp->fmuData->simulationInfo->realParameter[3], 0, &comp->fmuData->modelData->realParameterData[3].attribute.start);
  put_real_element(comp->fmuData->simulationInfo->realParameter[4], 0, &comp->fmuData->modelData->realParameterData[4].attribute.start);
}


// implementation of the Model Exchange functions
// Used to set the next time event, if any.
void eventUpdate(ModelInstance* comp, fmi2EventInfo* eventInfo) {
}

fmi2Real getReal(ModelInstance* comp, const fmi2ValueReference vr) {
  if (vr < 22) {
    return comp->fmuData->localData[0]->realVars[vr];
  }
  if (vr < 27) {
    return comp->fmuData->simulationInfo->realParameter[vr-22];
  }
  return NAN;
}

fmi2Status setReal(ModelInstance* comp, const fmi2ValueReference vr, const fmi2Real value) {
  // set start value attribute for all variable that has start value, till initialization mode
  if (vr < 22 && (comp->state == model_state_instantiated || comp->state == model_state_initialization_mode)) {
    put_real_element(value, 0, &comp->fmuData->modelData->realVarsData[vr].attribute.start);
  }
  if (vr < 22) {
    comp->fmuData->localData[0]->realVars[vr] = value;
    return fmi2OK;
  }
  if (vr < 27) {
    comp->fmuData->simulationInfo->realParameter[vr-22] = value;
    return fmi2OK;
  }
  return fmi2Error;
}

fmi2Integer getInteger(ModelInstance* comp, const fmi2ValueReference vr) {
  if (vr < 0) {
    return comp->fmuData->localData[0]->integerVars[vr];
  }
  if (vr < 0) {
    return comp->fmuData->simulationInfo->integerParameter[vr-0];
  }
  return 0;
}

fmi2Status setInteger(ModelInstance* comp, const fmi2ValueReference vr, const fmi2Integer value) {
  // set start value attribute for all variable that has start value, till initialization mode
  if (vr < 0 && (comp->state == model_state_instantiated || comp->state == model_state_initialization_mode)) {
    comp->fmuData->modelData->integerVarsData[vr].attribute.start = value;
  }
  if (vr < 0) {
    comp->fmuData->localData[0]->integerVars[vr] = value;
    return fmi2OK;
  }
  if (vr < 0) {
    comp->fmuData->simulationInfo->integerParameter[vr-0] = value;
    return fmi2OK;
  }
  return fmi2Error;
}
fmi2Boolean getBoolean(ModelInstance* comp, const fmi2ValueReference vr) {
  switch (vr) {
    default:
      return fmi2False;
  }
}

fmi2Status setBoolean(ModelInstance* comp, const fmi2ValueReference vr, const fmi2Boolean value) {
  switch (vr) {
    default:
      return fmi2Error;
  }
  return fmi2OK;
}

fmi2String getString(ModelInstance* comp, const fmi2ValueReference vr) {
  switch (vr) {
    default:
      return "";
  }
}

fmi2Status setString(ModelInstance* comp, const fmi2ValueReference vr, fmi2String value) {
  switch (vr) {
    default:
      return fmi2Error;
  }
  return fmi2OK;
}

fmi2Status setExternalFunction(ModelInstance* c, const fmi2ValueReference vr, const void* value){
  switch (vr) {
    default:
      return fmi2Error;
  }
  return fmi2OK;
}

/* function maps input references to a input index used in partialDerivatives */
fmi2ValueReference mapInputReference2InputNumber(const fmi2ValueReference vr) {
    switch (vr) {
      case 12: return 0; break;
      case 13: return 1; break;
      case 14: return 2; break;
      default:
        return -1;
    }
}
/* function maps output references to a input index used in partialDerivatives */
fmi2ValueReference mapOutputReference2OutputNumber(const fmi2ValueReference vr) {
    switch (vr) {
      case 16: return 0; break;
      case 17: return 1; break;
      case 18: return 2; break;
      case 19: return 3; break;
      case 20: return 4; break;
      case 21: return 5; break;
      default:
        return -1;
    }
}
/* function maps output references to an internal output Real derivatives */
fmi2ValueReference mapOutputReference2RealOutputDerivatives(const fmi2ValueReference vr) {
    switch (vr) {
      case 16: return -1; break;
      case 17: return -1; break;
      case 18: return -1; break;
      case 19: return -1; break;
      case 20: return -1; break;
      case 21: return -1; break;
      default:
        return -1;
    }
}
/* function maps initialUnknowns UnknownVars ValueReferences to an internal partial derivatives index */
fmi2ValueReference mapInitialUnknownsdependentIndex(const fmi2ValueReference vr) {
    switch (vr) {
      case 3: return 0; break;
      case 4: return 1; break;
      case 5: return 2; break;
      case 6: return 3; break;
      case 7: return 4; break;
      case 8: return 5; break;
      case 9: return 6; break;
      case 10: return 7; break;
      case 11: return 8; break;
      case 16: return 9; break;
      case 17: return 10; break;
      case 18: return 11; break;
      case 19: return 12; break;
      case 20: return 13; break;
      case 21: return 14; break;
      default:
        return -1;
    }
}
/* function maps initialUnknowns knownVars ValueReferences to an internal partial derivatives index */
fmi2ValueReference mapInitialUnknownsIndependentIndex(const fmi2ValueReference vr) {
    switch (vr) {
      case 0: return 0; break;
      case 1: return 1; break;
      case 2: return 2; break;
      case 12: return 3; break;
      case 13: return 4; break;
      case 14: return 5; break;
      case 22: return 6; break;
      case 23: return 7; break;
      case 24: return 8; break;
      case 25: return 9; break;
      case 26: return 10; break;
      default:
        return -1;
    }
}

