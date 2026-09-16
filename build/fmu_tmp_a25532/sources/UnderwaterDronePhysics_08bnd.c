/* update bound parameters and variable attributes (start, nominal, min, max) */
#include "UnderwaterDronePhysics_model.h"
#if defined(__cplusplus)
extern "C" {
#endif


/*
equation index: 44
type: SIMPLE_ASSIGN
$START.$outputAlias_battery = battery_max
*/
static void UnderwaterDronePhysics_eqFunction_44(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,44};
  ((modelica_real *)((data->modelData->realVarsData[0] /* $outputAlias_battery STATE(1) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */) = ((modelica_real *)((data->modelData->realVarsData[0] /* $outputAlias_battery STATE(1) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[0] /* $outputAlias_battery */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */));
  threadData->lastEquationSolved = 44;
}

/*
equation index: 45
type: SIMPLE_ASSIGN
$START.$outputAlias_z = start_z
*/
static void UnderwaterDronePhysics_eqFunction_45(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,45};
  ((modelica_real *)((data->modelData->realVarsData[6] /* $outputAlias_z STATE(1,$outputAlias_vz) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[9]] /* start_z PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[6]] /* $outputAlias_z STATE(1,$outputAlias_vz) */) = ((modelica_real *)((data->modelData->realVarsData[6] /* $outputAlias_z STATE(1,$outputAlias_vz) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[6] /* $outputAlias_z */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[6]] /* $outputAlias_z STATE(1,$outputAlias_vz) */));
  threadData->lastEquationSolved = 45;
}

/*
equation index: 46
type: SIMPLE_ASSIGN
$START.$outputAlias_y = start_y
*/
static void UnderwaterDronePhysics_eqFunction_46(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,46};
  ((modelica_real *)((data->modelData->realVarsData[5] /* $outputAlias_y STATE(1,$outputAlias_vy) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[8]] /* start_y PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_y STATE(1,$outputAlias_vy) */) = ((modelica_real *)((data->modelData->realVarsData[5] /* $outputAlias_y STATE(1,$outputAlias_vy) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[5] /* $outputAlias_y */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_y STATE(1,$outputAlias_vy) */));
  threadData->lastEquationSolved = 46;
}

/*
equation index: 47
type: SIMPLE_ASSIGN
$START.$outputAlias_x = start_x
*/
static void UnderwaterDronePhysics_eqFunction_47(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,47};
  ((modelica_real *)((data->modelData->realVarsData[4] /* $outputAlias_x STATE(1,$outputAlias_vx) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[7]] /* start_x PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_x STATE(1,$outputAlias_vx) */) = ((modelica_real *)((data->modelData->realVarsData[4] /* $outputAlias_x STATE(1,$outputAlias_vx) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[4] /* $outputAlias_x */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_x STATE(1,$outputAlias_vx) */));
  threadData->lastEquationSolved = 47;
}
OMC_DISABLE_OPT
int UnderwaterDronePhysics_updateBoundVariableAttributes(DATA *data, threadData_t *threadData)
{
  /* min ******************************************************** */
  infoStreamPrint(OMC_LOG_INIT, 1, "updating min-values");
  messageClose(OMC_LOG_INIT);
  
  /* max ******************************************************** */
  infoStreamPrint(OMC_LOG_INIT, 1, "updating max-values");
  messageClose(OMC_LOG_INIT);
  
  /* nominal **************************************************** */
  infoStreamPrint(OMC_LOG_INIT, 1, "updating nominal-values");
  messageClose(OMC_LOG_INIT);
  
  /* start ****************************************************** */
  infoStreamPrint(OMC_LOG_INIT, 1, "updating primary start-values");
  UnderwaterDronePhysics_eqFunction_44(data, threadData);
  UnderwaterDronePhysics_eqFunction_45(data, threadData);
  UnderwaterDronePhysics_eqFunction_46(data, threadData);
  UnderwaterDronePhysics_eqFunction_47(data, threadData);
  messageClose(OMC_LOG_INIT);
  
  return 0;
}

OMC_DISABLE_OPT
int UnderwaterDronePhysics_updateBoundParameters(DATA *data, threadData_t *threadData)
{
  return 0;
}

#if defined(__cplusplus)
}
#endif
