/* Main Simulation File */

#if defined(__cplusplus)
extern "C" {
#endif

#include "UnderwaterDronePhysics_model.h"
#include "simulation/solver/events.h"
#include "simulation/arrayIndex.h"



/* dummy VARINFO and FILEINFO */
const VAR_INFO dummyVAR_INFO = omc_dummyVarInfo;

int UnderwaterDronePhysics_input_function(DATA *data, threadData_t *threadData)
{
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[12]] /* ux variable */) = data->simulationInfo->inputVars[0];
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[13]] /* uy variable */) = data->simulationInfo->inputVars[1];
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[14]] /* uz variable */) = data->simulationInfo->inputVars[2];
  
  return 0;
}

int UnderwaterDronePhysics_input_function_init(DATA *data, threadData_t *threadData)
{
  data->simulationInfo->inputVars[0] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 12);
  data->simulationInfo->inputVars[1] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 13);
  data->simulationInfo->inputVars[2] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 14);
  
  return 0;
}

int UnderwaterDronePhysics_input_function_updateStartValues(DATA *data, threadData_t *threadData)
{
  assertStreamPrint(threadData, data->modelData->realVarsData[12].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[0], 0, &data->modelData->realVarsData[12].attribute.start);
  assertStreamPrint(threadData, data->modelData->realVarsData[13].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[1], 0, &data->modelData->realVarsData[13].attribute.start);
  assertStreamPrint(threadData, data->modelData->realVarsData[14].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[2], 0, &data->modelData->realVarsData[14].attribute.start);
  
  return 0;
}

int UnderwaterDronePhysics_inputNames(DATA *data, char ** names){
  names[0] = (char *) data->modelData->realVarsData[12].info.name;
  names[1] = (char *) data->modelData->realVarsData[13].info.name;
  names[2] = (char *) data->modelData->realVarsData[14].info.name;
  
  return 0;
}

int UnderwaterDronePhysics_data_function(DATA *data, threadData_t *threadData)
{
  return 0;
}

int UnderwaterDronePhysics_dataReconciliationInputNames(DATA *data, char ** names){
  
  return 0;
}

int UnderwaterDronePhysics_dataReconciliationUnmeasuredVariables(DATA *data, char ** names)
{
  
  return 0;
}

int UnderwaterDronePhysics_output_function(DATA *data, threadData_t *threadData)
{
  data->simulationInfo->outputVars[0] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* vx variable */);
  data->simulationInfo->outputVars[1] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[17]] /* vy variable */);
  data->simulationInfo->outputVars[2] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[18]] /* vz variable */);
  data->simulationInfo->outputVars[3] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[19]] /* x variable */);
  data->simulationInfo->outputVars[4] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[20]] /* y variable */);
  data->simulationInfo->outputVars[5] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* z variable */);
  
  return 0;
}

int UnderwaterDronePhysics_setc_function(DATA *data, threadData_t *threadData)
{
  
  return 0;
}

int UnderwaterDronePhysics_setb_function(DATA *data, threadData_t *threadData)
{
  
  return 0;
}


/*
equation index: 20
type: SIMPLE_ASSIGN
v_norm = sqrt($outputAlias_vx ^ 2.0 + $outputAlias_vy ^ 2.0 + $outputAlias_vz ^ 2.0)
*/
void UnderwaterDronePhysics_eqFunction_20(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,20};
  modelica_real tmp0;
  modelica_real tmp1;
  modelica_real tmp2;
  modelica_real tmp3;
  tmp0 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_vx STATE(1) */);
  tmp1 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vy STATE(1) */);
  tmp2 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vz STATE(1) */);
  tmp3 = (tmp0 * tmp0) + (tmp1 * tmp1) + (tmp2 * tmp2);
  if(!(tmp3 >= 0.0))
  {
    if (data->simulationInfo->noThrowAsserts) {
      FILE_INFO info = {"",0,0,0,0,0};
      infoStreamPrintWithEquationIndexes(OMC_LOG_ASSERT, info, 0, equationIndexes, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      data->simulationInfo->needToReThrow = 1;
    } else {
      FILE_INFO info = {"",0,0,0,0,0};
      omc_assert_warning(info, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      throwStreamPrintWithEquationIndexes(threadData, info, equationIndexes, "Model error: Argument of sqrt($outputAlias_vx ^ 2.0 + $outputAlias_vy ^ 2.0 + $outputAlias_vz ^ 2.0) was %g should be >= 0", tmp3);
    }
  }
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* v_norm variable */) = sqrt(tmp3);
  threadData->lastEquationSolved = 20;
}

/*
equation index: 21
type: SIMPLE_ASSIGN
$DER.$outputAlias_vx = (ux - kd * $outputAlias_vx * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_21(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,21};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[6]] /* der($outputAlias_vx) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[12]] /* ux variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_vx STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 21;
}

/*
equation index: 22
type: SIMPLE_ASSIGN
$DER.$outputAlias_vy = (uy - kd * $outputAlias_vy * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_22(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,22};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[7]] /* der($outputAlias_vy) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[13]] /* uy variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vy STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 22;
}

/*
equation index: 23
type: SIMPLE_ASSIGN
$DER.$outputAlias_vz = (uz - kd * $outputAlias_vz * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_23(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,23};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[8]] /* der($outputAlias_vz) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[14]] /* uz variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vz STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 23;
}

/*
equation index: 24
type: SIMPLE_ASSIGN
$DER.$outputAlias_x = $outputAlias_vx
*/
void UnderwaterDronePhysics_eqFunction_24(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,24};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[9]] /* der($outputAlias_x) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_vx STATE(1) */);
  threadData->lastEquationSolved = 24;
}

/*
equation index: 25
type: SIMPLE_ASSIGN
$DER.$outputAlias_y = $outputAlias_vy
*/
void UnderwaterDronePhysics_eqFunction_25(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,25};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[10]] /* der($outputAlias_y) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vy STATE(1) */);
  threadData->lastEquationSolved = 25;
}

/*
equation index: 26
type: SIMPLE_ASSIGN
$DER.$outputAlias_z = $outputAlias_vz
*/
void UnderwaterDronePhysics_eqFunction_26(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,26};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[11]] /* der($outputAlias_z) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vz STATE(1) */);
  threadData->lastEquationSolved = 26;
}

/*
equation index: 27
type: SIMPLE_ASSIGN
vz = $outputAlias_vz
*/
void UnderwaterDronePhysics_eqFunction_27(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,27};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[18]] /* vz variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vz STATE(1) */);
  threadData->lastEquationSolved = 27;
}

/*
equation index: 28
type: SIMPLE_ASSIGN
vy = $outputAlias_vy
*/
void UnderwaterDronePhysics_eqFunction_28(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,28};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[17]] /* vy variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vy STATE(1) */);
  threadData->lastEquationSolved = 28;
}

/*
equation index: 29
type: SIMPLE_ASSIGN
vx = $outputAlias_vx
*/
void UnderwaterDronePhysics_eqFunction_29(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,29};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* vx variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_vx STATE(1) */);
  threadData->lastEquationSolved = 29;
}

/*
equation index: 30
type: SIMPLE_ASSIGN
z = $outputAlias_z
*/
void UnderwaterDronePhysics_eqFunction_30(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,30};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* z variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_z STATE(1,$outputAlias_vz) */);
  threadData->lastEquationSolved = 30;
}

/*
equation index: 31
type: SIMPLE_ASSIGN
y = $outputAlias_y
*/
void UnderwaterDronePhysics_eqFunction_31(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,31};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[20]] /* y variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_y STATE(1,$outputAlias_vy) */);
  threadData->lastEquationSolved = 31;
}

/*
equation index: 32
type: SIMPLE_ASSIGN
x = $outputAlias_x
*/
void UnderwaterDronePhysics_eqFunction_32(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,32};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[19]] /* x variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_x STATE(1,$outputAlias_vx) */);
  threadData->lastEquationSolved = 32;
}

OMC_DISABLE_OPT
int UnderwaterDronePhysics_functionDAE(DATA *data, threadData_t *threadData)
{
  int equationIndexes[1] = {0};
#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_tick(SIM_TIMER_DAE);
#endif

  data->simulationInfo->needToIterate = 0;
  data->simulationInfo->discreteCall = 1;
  UnderwaterDronePhysics_functionLocalKnownVars(data, threadData);
  static void (*const eqFunctions[13])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_20,
    UnderwaterDronePhysics_eqFunction_21,
    UnderwaterDronePhysics_eqFunction_22,
    UnderwaterDronePhysics_eqFunction_23,
    UnderwaterDronePhysics_eqFunction_24,
    UnderwaterDronePhysics_eqFunction_25,
    UnderwaterDronePhysics_eqFunction_26,
    UnderwaterDronePhysics_eqFunction_27,
    UnderwaterDronePhysics_eqFunction_28,
    UnderwaterDronePhysics_eqFunction_29,
    UnderwaterDronePhysics_eqFunction_30,
    UnderwaterDronePhysics_eqFunction_31,
    UnderwaterDronePhysics_eqFunction_32
  };
  
  for (int id = 0; id < 13; id++) {
    eqFunctions[id](data, threadData);
  }
  data->simulationInfo->discreteCall = 0;
  
#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_accumulate(SIM_TIMER_DAE);
#endif
  return 0;
}


int UnderwaterDronePhysics_functionLocalKnownVars(DATA *data, threadData_t *threadData)
{
  
  return 0;
}

/* forwarded equations */
extern void UnderwaterDronePhysics_eqFunction_20(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_21(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_22(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_23(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_24(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_25(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_26(DATA* data, threadData_t *threadData);

static void functionODE_system0(DATA *data, threadData_t *threadData)
{
  static void (*const eqFunctions[7])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_20,
    UnderwaterDronePhysics_eqFunction_21,
    UnderwaterDronePhysics_eqFunction_22,
    UnderwaterDronePhysics_eqFunction_23,
    UnderwaterDronePhysics_eqFunction_24,
    UnderwaterDronePhysics_eqFunction_25,
    UnderwaterDronePhysics_eqFunction_26
  };
  
  if (data->simulationInfo->evalSelection) {
    for (int i = 0; i < data->simulationInfo->evalSelection->n; i++) {
      int id = data->simulationInfo->evalSelection->idx[i];
      eqFunctions[id](data, threadData);
    }
  } else {
    for (int id = 0; id < 7; id++) {
      eqFunctions[id](data, threadData);
    }
  }
}

int UnderwaterDronePhysics_functionODE(DATA *data, threadData_t *threadData)
{
#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_tick(SIM_TIMER_FUNCTION_ODE);
#endif

  
  data->simulationInfo->callStatistics.functionODE++;
  
  UnderwaterDronePhysics_functionLocalKnownVars(data, threadData);
  functionODE_system0(data, threadData);

#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_accumulate(SIM_TIMER_FUNCTION_ODE);
#endif

  return 0;
}

void UnderwaterDronePhysics_ODE_DAG(DATA* data, threadData_t* threadData)
{
  const size_t eqMap[] = {20, 21, 22, 23, 24, 25, 26};
  buildEvalDAG_ODE(data->modelData, sizeof(eqMap)/sizeof(size_t), eqMap);
}

/* forward the main in the simulation runtime */
extern int _main_SimulationRuntime(int argc, char **argv, DATA *data, threadData_t *threadData);
extern int _main_OptimizationRuntime(int argc, char **argv, DATA *data, threadData_t *threadData);

#include "UnderwaterDronePhysics_12jac.h"
#include "UnderwaterDronePhysics_13opt.h"

struct OpenModelicaGeneratedFunctionCallbacks UnderwaterDronePhysics_callback = {
  NULL,    /* performSimulation */
  NULL,    /* performQSSSimulation */
  NULL,    /* updateContinuousSystem */
  UnderwaterDronePhysics_callExternalObjectDestructors,    /* callExternalObjectDestructors */
  NULL,    /* initialNonLinearSystem */
  NULL,    /* initialLinearSystem */
  NULL,    /* initialMixedSystem */
  #if !defined(OMC_NO_STATESELECTION)
  UnderwaterDronePhysics_initializeStateSets,
  #else
  NULL,
  #endif    /* initializeStateSets */
  UnderwaterDronePhysics_initializeDAEmodeData,
  UnderwaterDronePhysics_ODE_DAG,
  UnderwaterDronePhysics_functionODE,
  UnderwaterDronePhysics_functionAlgebraics,
  UnderwaterDronePhysics_functionDAE,
  UnderwaterDronePhysics_functionLocalKnownVars,
  UnderwaterDronePhysics_input_function,
  UnderwaterDronePhysics_input_function_init,
  UnderwaterDronePhysics_input_function_updateStartValues,
  UnderwaterDronePhysics_data_function,
  UnderwaterDronePhysics_output_function,
  UnderwaterDronePhysics_setc_function,
  UnderwaterDronePhysics_setb_function,
  UnderwaterDronePhysics_function_storeDelayed,
  UnderwaterDronePhysics_function_storeSpatialDistribution,
  UnderwaterDronePhysics_function_initSpatialDistribution,
  UnderwaterDronePhysics_updateBoundVariableAttributes,
  UnderwaterDronePhysics_functionInitialEquations,
  GLOBAL_EQUIDISTANT_HOMOTOPY,
  NULL,
  UnderwaterDronePhysics_functionRemovedInitialEquations,
  UnderwaterDronePhysics_updateBoundParameters,
  UnderwaterDronePhysics_checkForAsserts,
  UnderwaterDronePhysics_function_ZeroCrossingsEquations,
  UnderwaterDronePhysics_function_ZeroCrossings,
  UnderwaterDronePhysics_function_updateRelations,
  UnderwaterDronePhysics_zeroCrossingDescription,
  UnderwaterDronePhysics_relationDescription,
  UnderwaterDronePhysics_function_initSample,
  UnderwaterDronePhysics_INDEX_JAC_A,
  UnderwaterDronePhysics_INDEX_JAC_ADJ,
  UnderwaterDronePhysics_INDEX_JAC_B,
  UnderwaterDronePhysics_INDEX_JAC_C,
  UnderwaterDronePhysics_INDEX_JAC_D,
  UnderwaterDronePhysics_INDEX_JAC_F,
  UnderwaterDronePhysics_INDEX_JAC_H,
  UnderwaterDronePhysics_initialAnalyticJacobianA,
  UnderwaterDronePhysics_initialAnalyticJacobianADJ,
  UnderwaterDronePhysics_initialAnalyticJacobianB,
  UnderwaterDronePhysics_initialAnalyticJacobianC,
  UnderwaterDronePhysics_initialAnalyticJacobianD,
  UnderwaterDronePhysics_initialAnalyticJacobianF,
  UnderwaterDronePhysics_initialAnalyticJacobianH,
  UnderwaterDronePhysics_functionJacA_column,
  UnderwaterDronePhysics_functionJacADJ_column,
  UnderwaterDronePhysics_functionJacB_column,
  UnderwaterDronePhysics_functionJacC_column,
  UnderwaterDronePhysics_functionJacD_column,
  UnderwaterDronePhysics_functionJacF_column,
  UnderwaterDronePhysics_functionJacH_column,
  UnderwaterDronePhysics_JacA_DAG,
  UnderwaterDronePhysics_linear_model_frame,
  UnderwaterDronePhysics_linear_model_datarecovery_frame,
  UnderwaterDronePhysics_mayer,
  UnderwaterDronePhysics_lagrange,
  UnderwaterDronePhysics_getInputVarIndicesInOptimization,
  UnderwaterDronePhysics_pickUpBoundsForInputsInOptimization,
  UnderwaterDronePhysics_setInputData,
  UnderwaterDronePhysics_getTimeGrid,
  UnderwaterDronePhysics_symbolicInlineSystem,
  UnderwaterDronePhysics_function_initSynchronous,
  UnderwaterDronePhysics_function_updateSynchronous,
  UnderwaterDronePhysics_function_equationsSynchronous,
  UnderwaterDronePhysics_inputNames,
  UnderwaterDronePhysics_dataReconciliationInputNames,
  UnderwaterDronePhysics_dataReconciliationUnmeasuredVariables,
  UnderwaterDronePhysics_read_simulation_info,
  UnderwaterDronePhysics_read_input_fmu,
  NULL,
  NULL,
  -1,
  NULL,
  NULL,
  -1

};

#define _OMC_LIT_RESOURCE_0_name_data "Complex"
#define _OMC_LIT_RESOURCE_0_dir_data "C:/Users/maxbu/AppData/Roaming/.openmodelica/libraries/Complex 4.1.0+maint.om"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_0_name,7,_OMC_LIT_RESOURCE_0_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_0_dir,77,_OMC_LIT_RESOURCE_0_dir_data);

#define _OMC_LIT_RESOURCE_1_name_data "DroneSwarm"
#define _OMC_LIT_RESOURCE_1_dir_data "C:/Users/maxbu/Desktop/uni/Verifica e validazione s/progetto/controllore"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_1_name,10,_OMC_LIT_RESOURCE_1_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_1_dir,72,_OMC_LIT_RESOURCE_1_dir_data);

#define _OMC_LIT_RESOURCE_2_name_data "Modelica"
#define _OMC_LIT_RESOURCE_2_dir_data "C:/Users/maxbu/AppData/Roaming/.openmodelica/libraries/Modelica 4.1.0+maint.om"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_2_name,8,_OMC_LIT_RESOURCE_2_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_2_dir,78,_OMC_LIT_RESOURCE_2_dir_data);

#define _OMC_LIT_RESOURCE_3_name_data "ModelicaServices"
#define _OMC_LIT_RESOURCE_3_dir_data "C:/Users/maxbu/AppData/Roaming/.openmodelica/libraries/ModelicaServices 4.1.0+maint.om"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_3_name,16,_OMC_LIT_RESOURCE_3_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_3_dir,86,_OMC_LIT_RESOURCE_3_dir_data);

#define _OMC_LIT_RESOURCE_4_name_data "UnderwaterDrone"
#define _OMC_LIT_RESOURCE_4_dir_data "C:/Users/maxbu/Desktop/uni/Verifica e validazione s/progetto/controllore"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_4_name,15,_OMC_LIT_RESOURCE_4_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_4_dir,72,_OMC_LIT_RESOURCE_4_dir_data);

#define _OMC_LIT_RESOURCE_5_name_data "UnderwaterDronePhysics"
#define _OMC_LIT_RESOURCE_5_dir_data "C:/Users/maxbu/Desktop/uni/Verifica e validazione s/progetto/controllore"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_5_name,22,_OMC_LIT_RESOURCE_5_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_5_dir,72,_OMC_LIT_RESOURCE_5_dir_data);

static const MMC_DEFSTRUCTLIT(_OMC_LIT_RESOURCES,12,MMC_ARRAY_TAG) {MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_0_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_0_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_1_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_1_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_2_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_2_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_3_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_3_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_4_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_4_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_5_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_5_dir)}};
void UnderwaterDronePhysics_setupDataStruc(DATA *data, threadData_t *threadData)
{
  assertStreamPrint(threadData,0!=data, "Error while initialize Data");
  threadData->localRoots[LOCAL_ROOT_SIMULATION_DATA] = data;
  data->callback = &UnderwaterDronePhysics_callback;
  OpenModelica_updateUriMapping(threadData, MMC_REFSTRUCTLIT(_OMC_LIT_RESOURCES));
  data->modelData->modelName = "UnderwaterDronePhysics";
  data->modelData->modelFilePrefix = "UnderwaterDronePhysics";
  data->modelData->modelFileName = "UnderwaterDrone2.mo";
  data->modelData->resultFileName = NULL;
  data->modelData->modelDir = "C:/Users/maxbu/Desktop/uni/Verifica e validazione s/progetto/controllore";
  data->modelData->modelGUID = "{00a785b0-ed17-4d82-ab03-1ee03c1d9b9e}";
  data->modelData->initXMLData = NULL;
  data->modelData->modelDataXml.infoXMLData = NULL;
  GC_asprintf(&data->modelData->modelDataXml.fileName, "%s/UnderwaterDronePhysics_info.json", data->modelData->resourcesDir);
  data->modelData->runTestsuite = 0;
  data->modelData->nStatesArray = 6;
  data->modelData->nDiscreteReal = 0;
  data->modelData->nVariablesRealArray = 22;
  data->modelData->nVariablesIntegerArray = 0;
  data->modelData->nVariablesBooleanArray = 0;
  data->modelData->nVariablesStringArray = 0;
  data->modelData->nParametersRealArray = 5;
  data->modelData->nParametersIntegerArray = 0;
  data->modelData->nParametersBooleanArray = 0;
  data->modelData->nParametersStringArray = 0;
  data->modelData->nParametersReal = 5;
  data->modelData->nParametersInteger = 0;
  data->modelData->nParametersBoolean = 0;
  data->modelData->nParametersString = 0;
  data->modelData->nAliasRealArray = 0;
  data->modelData->nAliasIntegerArray = 0;
  data->modelData->nAliasBooleanArray = 0;
  data->modelData->nAliasStringArray = 0;
  data->modelData->nInputVars = 3;
  data->modelData->nOutputVars = 6;
  data->modelData->nZeroCrossings = 0;
  data->modelData->nSamples = 0;
  data->modelData->nRelations = 0;
  data->modelData->nMathEvents = 0;
  data->modelData->nExtObjs = 0;
  data->modelData->modelDataXml.modelInfoXmlLength = 0;
  data->modelData->modelDataXml.nFunctions = 0;
  data->modelData->modelDataXml.nProfileBlocks = 0;
  data->modelData->modelDataXml.nEquations = 36;
  data->modelData->nMixedSystems = 0;
  data->modelData->nLinearSystems = 0;
  data->modelData->nNonLinearSystems = 0;
  data->modelData->nStateSets = 0;
  data->modelData->nJacobians = 7;
  data->modelData->nOptimizeConstraints = 0;
  data->modelData->nOptimizeFinalConstraints = 0;
  data->modelData->nDelayExpressions = 0;
  data->modelData->nBaseClocks = 0;
  data->modelData->nSpatialDistributions = 0;
  data->modelData->nSensitivityVars = 0;
  data->modelData->nSensitivityParamVars = 0;
  data->modelData->nSetcVars = 0;
  data->modelData->ndataReconVars = 0;
  data->modelData->nSetbVars = 0;
  data->modelData->nRelatedBoundaryConditions = 0;
  data->modelData->linearizationDumpLanguage = OMC_LINEARIZE_DUMP_LANGUAGE_MODELICA;
}

static int rml_execution_failed()
{
  fflush(NULL);
  fprintf(stderr, "Execution failed!\n");
  fflush(NULL);
  return 1;
}

