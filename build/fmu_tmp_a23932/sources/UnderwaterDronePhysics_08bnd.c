/* update bound parameters and variable attributes (start, nominal, min, max) */
#include "UnderwaterDronePhysics_model.h"
#if defined(__cplusplus)
extern "C" {
#endif


/*
equation index: 33
type: SIMPLE_ASSIGN
$START.$outputAlias_z = start_z
*/
static void UnderwaterDronePhysics_eqFunction_33(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,33};
  ((modelica_real *)((data->modelData->realVarsData[5] /* $outputAlias_z STATE(1,$outputAlias_vz) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[4]] /* start_z PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_z STATE(1,$outputAlias_vz) */) = ((modelica_real *)((data->modelData->realVarsData[5] /* $outputAlias_z STATE(1,$outputAlias_vz) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[5] /* $outputAlias_z */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_z STATE(1,$outputAlias_vz) */));
  threadData->lastEquationSolved = 33;
}

/*
equation index: 34
type: SIMPLE_ASSIGN
$START.$outputAlias_y = start_y
*/
static void UnderwaterDronePhysics_eqFunction_34(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,34};
  ((modelica_real *)((data->modelData->realVarsData[4] /* $outputAlias_y STATE(1,$outputAlias_vy) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[3]] /* start_y PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_y STATE(1,$outputAlias_vy) */) = ((modelica_real *)((data->modelData->realVarsData[4] /* $outputAlias_y STATE(1,$outputAlias_vy) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[4] /* $outputAlias_y */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_y STATE(1,$outputAlias_vy) */));
  threadData->lastEquationSolved = 34;
}

/*
equation index: 35
type: SIMPLE_ASSIGN
$START.$outputAlias_x = start_x
*/
static void UnderwaterDronePhysics_eqFunction_35(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,35};
  ((modelica_real *)((data->modelData->realVarsData[3] /* $outputAlias_x STATE(1,$outputAlias_vx) */).attribute .start.data))[0] = (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[2]] /* start_x PARAM */);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_x STATE(1,$outputAlias_vx) */) = ((modelica_real *)((data->modelData->realVarsData[3] /* $outputAlias_x STATE(1,$outputAlias_vx) */).attribute .start.data))[0];
  infoStreamPrint(OMC_LOG_INIT_V, 0,
                  "updated start value: %s(start=%g)",
                  data->modelData->realVarsData[3] /* $outputAlias_x */ .info.name,
                  (modelica_real) (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_x STATE(1,$outputAlias_vx) */));
  threadData->lastEquationSolved = 35;
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
  UnderwaterDronePhysics_eqFunction_33(data, threadData);
  UnderwaterDronePhysics_eqFunction_34(data, threadData);
  UnderwaterDronePhysics_eqFunction_35(data, threadData);
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
