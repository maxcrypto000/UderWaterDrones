/* Initialization */
#include "UnderwaterDronePhysics_model.h"
#include "UnderwaterDronePhysics_11mix.h"
#include "UnderwaterDronePhysics_12jac.h"
#if defined(__cplusplus)
extern "C" {
#endif

void UnderwaterDronePhysics_functionInitialEquations_0(DATA *data, threadData_t *threadData);

/*
equation index: 1
type: SIMPLE_ASSIGN
$outputAlias_vx = $START.$outputAlias_vx
*/
void UnderwaterDronePhysics_eqFunction_1(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,1};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_vx STATE(1) */) = ((modelica_real *)((data->modelData->realVarsData[0] /* $outputAlias_vx STATE(1) */).attribute .start.data))[0];
  threadData->lastEquationSolved = 1;
}
extern void UnderwaterDronePhysics_eqFunction_29(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_24(DATA *data, threadData_t *threadData);


/*
equation index: 4
type: SIMPLE_ASSIGN
$outputAlias_vy = $START.$outputAlias_vy
*/
void UnderwaterDronePhysics_eqFunction_4(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,4};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vy STATE(1) */) = ((modelica_real *)((data->modelData->realVarsData[1] /* $outputAlias_vy STATE(1) */).attribute .start.data))[0];
  threadData->lastEquationSolved = 4;
}
extern void UnderwaterDronePhysics_eqFunction_28(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_25(DATA *data, threadData_t *threadData);


/*
equation index: 7
type: SIMPLE_ASSIGN
$outputAlias_vz = $START.$outputAlias_vz
*/
void UnderwaterDronePhysics_eqFunction_7(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,7};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vz STATE(1) */) = ((modelica_real *)((data->modelData->realVarsData[2] /* $outputAlias_vz STATE(1) */).attribute .start.data))[0];
  threadData->lastEquationSolved = 7;
}
extern void UnderwaterDronePhysics_eqFunction_27(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_26(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_20(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_23(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_22(DATA *data, threadData_t *threadData);

extern void UnderwaterDronePhysics_eqFunction_21(DATA *data, threadData_t *threadData);


/*
equation index: 14
type: SIMPLE_ASSIGN
$outputAlias_z = start_z
*/
void UnderwaterDronePhysics_eqFunction_14(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,14};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_z STATE(1,$outputAlias_vz) */) = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[4]] /* start_z PARAM */);
  threadData->lastEquationSolved = 14;
}
extern void UnderwaterDronePhysics_eqFunction_30(DATA *data, threadData_t *threadData);


/*
equation index: 16
type: SIMPLE_ASSIGN
$outputAlias_y = start_y
*/
void UnderwaterDronePhysics_eqFunction_16(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,16};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_y STATE(1,$outputAlias_vy) */) = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[3]] /* start_y PARAM */);
  threadData->lastEquationSolved = 16;
}
extern void UnderwaterDronePhysics_eqFunction_31(DATA *data, threadData_t *threadData);


/*
equation index: 18
type: SIMPLE_ASSIGN
$outputAlias_x = start_x
*/
void UnderwaterDronePhysics_eqFunction_18(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,18};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_x STATE(1,$outputAlias_vx) */) = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[2]] /* start_x PARAM */);
  threadData->lastEquationSolved = 18;
}
extern void UnderwaterDronePhysics_eqFunction_32(DATA *data, threadData_t *threadData);

OMC_DISABLE_OPT
void UnderwaterDronePhysics_functionInitialEquations_0(DATA *data, threadData_t *threadData)
{
  static void (*const eqFunctions[19])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_1,
    UnderwaterDronePhysics_eqFunction_29,
    UnderwaterDronePhysics_eqFunction_24,
    UnderwaterDronePhysics_eqFunction_4,
    UnderwaterDronePhysics_eqFunction_28,
    UnderwaterDronePhysics_eqFunction_25,
    UnderwaterDronePhysics_eqFunction_7,
    UnderwaterDronePhysics_eqFunction_27,
    UnderwaterDronePhysics_eqFunction_26,
    UnderwaterDronePhysics_eqFunction_20,
    UnderwaterDronePhysics_eqFunction_23,
    UnderwaterDronePhysics_eqFunction_22,
    UnderwaterDronePhysics_eqFunction_21,
    UnderwaterDronePhysics_eqFunction_14,
    UnderwaterDronePhysics_eqFunction_30,
    UnderwaterDronePhysics_eqFunction_16,
    UnderwaterDronePhysics_eqFunction_31,
    UnderwaterDronePhysics_eqFunction_18,
    UnderwaterDronePhysics_eqFunction_32
  };
  
  for (int id = 0; id < 19; id++) {
    eqFunctions[id](data, threadData);
  }
}

int UnderwaterDronePhysics_functionInitialEquations(DATA *data, threadData_t *threadData)
{
  data->simulationInfo->discreteCall = 1;
  UnderwaterDronePhysics_functionInitialEquations_0(data, threadData);
  data->simulationInfo->discreteCall = 0;
  
  return 0;
}

/* No UnderwaterDronePhysics_functionInitialEquations_lambda0 function */

int UnderwaterDronePhysics_functionRemovedInitialEquations(DATA *data, threadData_t *threadData)
{
  const int *equationIndexes = NULL;
  double res = 0.0;

  
  return 0;
}


#if defined(__cplusplus)
}
#endif
