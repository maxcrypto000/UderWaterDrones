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
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[18]] /* ux variable */) = data->simulationInfo->inputVars[0];
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[19]] /* uy variable */) = data->simulationInfo->inputVars[1];
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[20]] /* uz variable */) = data->simulationInfo->inputVars[2];
  
  return 0;
}

int UnderwaterDronePhysics_input_function_init(DATA *data, threadData_t *threadData)
{
  data->simulationInfo->inputVars[0] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 18);
  data->simulationInfo->inputVars[1] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 19);
  data->simulationInfo->inputVars[2] = getStartFromScalarIdx(data->simulationInfo, data->modelData, VAR_TYPE_REAL, VAR_KIND_VARIABLE, 20);
  
  return 0;
}

int UnderwaterDronePhysics_input_function_updateStartValues(DATA *data, threadData_t *threadData)
{
  assertStreamPrint(threadData, data->modelData->realVarsData[18].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[0], 0, &data->modelData->realVarsData[18].attribute.start);
  assertStreamPrint(threadData, data->modelData->realVarsData[19].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[1], 0, &data->modelData->realVarsData[19].attribute.start);
  assertStreamPrint(threadData, data->modelData->realVarsData[20].dimension.numberOfDimensions == 0, "Handling of array variables not yet implemetned.");
  put_real_element(data->simulationInfo->inputVars[2], 0, &data->modelData->realVarsData[20].attribute.start);
  
  return 0;
}

int UnderwaterDronePhysics_inputNames(DATA *data, char ** names){
  names[0] = (char *) data->modelData->realVarsData[18].info.name;
  names[1] = (char *) data->modelData->realVarsData[19].info.name;
  names[2] = (char *) data->modelData->realVarsData[20].info.name;
  
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
  data->simulationInfo->outputVars[0] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[14]] /* battery variable */);
  data->simulationInfo->outputVars[1] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[22]] /* vx variable */);
  data->simulationInfo->outputVars[2] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[23]] /* vy variable */);
  data->simulationInfo->outputVars[3] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[24]] /* vz variable */);
  data->simulationInfo->outputVars[4] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[25]] /* x variable */);
  data->simulationInfo->outputVars[5] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[26]] /* y variable */);
  data->simulationInfo->outputVars[6] = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[27]] /* z variable */);
  
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
equation index: 26
type: SIMPLE_ASSIGN
v_norm = sqrt($outputAlias_vx ^ 2.0 + $outputAlias_vy ^ 2.0 + $outputAlias_vz ^ 2.0)
*/
void UnderwaterDronePhysics_eqFunction_26(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,26};
  modelica_real tmp0;
  modelica_real tmp1;
  modelica_real tmp2;
  modelica_real tmp3;
  tmp0 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vx STATE(1) */);
  tmp1 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vy STATE(1) */);
  tmp2 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_vz STATE(1) */);
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
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* v_norm variable */) = sqrt(tmp3);
  threadData->lastEquationSolved = 26;
}

/*
equation index: 27
type: SIMPLE_ASSIGN
$DER.$outputAlias_vx = (ux - kd * $outputAlias_vx * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_27(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,27};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[8]] /* der($outputAlias_vx) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[18]] /* ux variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[3]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vx STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[4]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 27;
}

/*
equation index: 28
type: SIMPLE_ASSIGN
$DER.$outputAlias_vy = (uy - kd * $outputAlias_vy * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_28(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,28};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[9]] /* der($outputAlias_vy) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[19]] /* uy variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[3]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vy STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[4]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 28;
}

/*
equation index: 29
type: SIMPLE_ASSIGN
$DER.$outputAlias_vz = (uz - kd * $outputAlias_vz * v_norm) / m
*/
void UnderwaterDronePhysics_eqFunction_29(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,29};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[10]] /* der($outputAlias_vz) STATE_DER */) = DIVISION_SIM((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[20]] /* uz variable */) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[3]] /* kd PARAM */)) * (((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_vz STATE(1) */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[21]] /* v_norm variable */)))),(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[4]] /* m PARAM */),"m",equationIndexes);
  threadData->lastEquationSolved = 29;
}

/*
equation index: 30
type: SIMPLE_ASSIGN
u_norm = sqrt(ux ^ 2.0 + uy ^ 2.0 + uz ^ 2.0)
*/
void UnderwaterDronePhysics_eqFunction_30(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,30};
  modelica_real tmp4;
  modelica_real tmp5;
  modelica_real tmp6;
  modelica_real tmp7;
  tmp4 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[18]] /* ux variable */);
  tmp5 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[19]] /* uy variable */);
  tmp6 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[20]] /* uz variable */);
  tmp7 = (tmp4 * tmp4) + (tmp5 * tmp5) + (tmp6 * tmp6);
  if(!(tmp7 >= 0.0))
  {
    if (data->simulationInfo->noThrowAsserts) {
      FILE_INFO info = {"",0,0,0,0,0};
      infoStreamPrintWithEquationIndexes(OMC_LOG_ASSERT, info, 0, equationIndexes, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      data->simulationInfo->needToReThrow = 1;
    } else {
      FILE_INFO info = {"",0,0,0,0,0};
      omc_assert_warning(info, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      throwStreamPrintWithEquationIndexes(threadData, info, equationIndexes, "Model error: Argument of sqrt(ux ^ 2.0 + uy ^ 2.0 + uz ^ 2.0) was %g should be >= 0", tmp7);
    }
  }
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[17]] /* u_norm variable */) = sqrt(tmp7);
  threadData->lastEquationSolved = 30;
}

/*
equation index: 31
type: SIMPLE_ASSIGN
distance_to_base = sqrt(($outputAlias_x - start_x) ^ 2.0 + ($outputAlias_y - start_y) ^ 2.0 + ($outputAlias_z - start_z) ^ 2.0)
*/
void UnderwaterDronePhysics_eqFunction_31(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,31};
  modelica_real tmp8;
  modelica_real tmp9;
  modelica_real tmp10;
  modelica_real tmp11;
  tmp8 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_x STATE(1,$outputAlias_vx) */) - (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[7]] /* start_x PARAM */);
  tmp9 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_y STATE(1,$outputAlias_vy) */) - (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[8]] /* start_y PARAM */);
  tmp10 = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[6]] /* $outputAlias_z STATE(1,$outputAlias_vz) */) - (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[9]] /* start_z PARAM */);
  tmp11 = (tmp8 * tmp8) + (tmp9 * tmp9) + (tmp10 * tmp10);
  if(!(tmp11 >= 0.0))
  {
    if (data->simulationInfo->noThrowAsserts) {
      FILE_INFO info = {"",0,0,0,0,0};
      infoStreamPrintWithEquationIndexes(OMC_LOG_ASSERT, info, 0, equationIndexes, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      data->simulationInfo->needToReThrow = 1;
    } else {
      FILE_INFO info = {"",0,0,0,0,0};
      omc_assert_warning(info, "The following assertion has been violated %sat time %f", initial() ? "during initialization " : "", data->localData[0]->timeValue);
      throwStreamPrintWithEquationIndexes(threadData, info, equationIndexes, "Model error: Argument of sqrt(($outputAlias_x - start_x) ^ 2.0 + ($outputAlias_y - start_y) ^ 2.0 + ($outputAlias_z - start_z) ^ 2.0) was %g should be >= 0", tmp11);
    }
  }
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* distance_to_base variable */) = sqrt(tmp11);
  threadData->lastEquationSolved = 31;
}

/*
equation index: 32
type: SIMPLE_ASSIGN
net_charge_rate = if distance_to_base <= base_tolerance then recharge_rate else (-idle_drain) - motor_drain_coeff * u_norm
*/
void UnderwaterDronePhysics_eqFunction_32(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,32};
  modelica_boolean tmp12;
  modelica_real tmp13;
  modelica_real tmp14;
  tmp13 = 1.0;
  tmp14 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */));
  relationhysteresis(data, &tmp12, (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* distance_to_base variable */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */), tmp13, tmp14, 0, LessEq, LessEqZC);
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */) = (tmp12?(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[6]] /* recharge_rate PARAM */):(-(data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[2]] /* idle_drain PARAM */)) - (((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[5]] /* motor_drain_coeff PARAM */)) * ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[17]] /* u_norm variable */))));
  threadData->lastEquationSolved = 32;
}

/*
equation index: 33
type: SIMPLE_ASSIGN
$DER.$outputAlias_battery = if $outputAlias_battery >= battery_max and net_charge_rate > 0.0 then 0.0 else if $outputAlias_battery <= 0.0 and net_charge_rate < 0.0 then 0.0 else net_charge_rate
*/
void UnderwaterDronePhysics_eqFunction_33(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,33};
  modelica_boolean tmp15;
  modelica_real tmp16;
  modelica_real tmp17;
  modelica_boolean tmp18;
  modelica_real tmp19;
  modelica_real tmp20;
  modelica_boolean tmp21;
  modelica_real tmp22;
  modelica_real tmp23;
  modelica_boolean tmp24;
  modelica_real tmp25;
  modelica_real tmp26;
  modelica_boolean tmp27;
  modelica_real tmp28;
  tmp16 = 1.0;
  tmp17 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */));
  relationhysteresis(data, &tmp15, (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */), tmp16, tmp17, 1, GreaterEq, GreaterEqZC);
  tmp19 = 1.0;
  tmp20 = 0.0;
  relationhysteresis(data, &tmp18, (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp19, tmp20, 2, Greater, GreaterZC);
  tmp27 = (modelica_boolean)(tmp15 && tmp18);
  if(tmp27)
  {
    tmp28 = 0.0;
  }
  else
  {
    tmp22 = 1.0;
    tmp23 = 0.0;
    relationhysteresis(data, &tmp21, (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), 0.0, tmp22, tmp23, 3, LessEq, LessEqZC);
    tmp25 = 1.0;
    tmp26 = 0.0;
    relationhysteresis(data, &tmp24, (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp25, tmp26, 4, Less, LessZC);
    tmp28 = ((tmp21 && tmp24)?0.0:(data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */));
  }
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[7]] /* der($outputAlias_battery) STATE_DER */) = tmp28;
  threadData->lastEquationSolved = 33;
}

/*
equation index: 34
type: SIMPLE_ASSIGN
$DER.$outputAlias_x = $outputAlias_vx
*/
void UnderwaterDronePhysics_eqFunction_34(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,34};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[11]] /* der($outputAlias_x) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vx STATE(1) */);
  threadData->lastEquationSolved = 34;
}

/*
equation index: 35
type: SIMPLE_ASSIGN
$DER.$outputAlias_y = $outputAlias_vy
*/
void UnderwaterDronePhysics_eqFunction_35(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,35};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[12]] /* der($outputAlias_y) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vy STATE(1) */);
  threadData->lastEquationSolved = 35;
}

/*
equation index: 36
type: SIMPLE_ASSIGN
$DER.$outputAlias_z = $outputAlias_vz
*/
void UnderwaterDronePhysics_eqFunction_36(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,36};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[13]] /* der($outputAlias_z) STATE_DER */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_vz STATE(1) */);
  threadData->lastEquationSolved = 36;
}

/*
equation index: 37
type: SIMPLE_ASSIGN
battery = $outputAlias_battery
*/
void UnderwaterDronePhysics_eqFunction_37(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,37};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[14]] /* battery variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */);
  threadData->lastEquationSolved = 37;
}

/*
equation index: 38
type: SIMPLE_ASSIGN
vz = $outputAlias_vz
*/
void UnderwaterDronePhysics_eqFunction_38(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,38};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[24]] /* vz variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[3]] /* $outputAlias_vz STATE(1) */);
  threadData->lastEquationSolved = 38;
}

/*
equation index: 39
type: SIMPLE_ASSIGN
vy = $outputAlias_vy
*/
void UnderwaterDronePhysics_eqFunction_39(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,39};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[23]] /* vy variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[2]] /* $outputAlias_vy STATE(1) */);
  threadData->lastEquationSolved = 39;
}

/*
equation index: 40
type: SIMPLE_ASSIGN
vx = $outputAlias_vx
*/
void UnderwaterDronePhysics_eqFunction_40(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,40};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[22]] /* vx variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[1]] /* $outputAlias_vx STATE(1) */);
  threadData->lastEquationSolved = 40;
}

/*
equation index: 41
type: SIMPLE_ASSIGN
z = $outputAlias_z
*/
void UnderwaterDronePhysics_eqFunction_41(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,41};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[27]] /* z variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[6]] /* $outputAlias_z STATE(1,$outputAlias_vz) */);
  threadData->lastEquationSolved = 41;
}

/*
equation index: 42
type: SIMPLE_ASSIGN
y = $outputAlias_y
*/
void UnderwaterDronePhysics_eqFunction_42(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,42};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[26]] /* y variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[5]] /* $outputAlias_y STATE(1,$outputAlias_vy) */);
  threadData->lastEquationSolved = 42;
}

/*
equation index: 43
type: SIMPLE_ASSIGN
x = $outputAlias_x
*/
void UnderwaterDronePhysics_eqFunction_43(DATA *data, threadData_t *threadData)
{
  const int equationIndexes[2] = {1,43};
  (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[25]] /* x variable */) = (data->localData[0]->realVars[data->simulationInfo->realVarsIndex[4]] /* $outputAlias_x STATE(1,$outputAlias_vx) */);
  threadData->lastEquationSolved = 43;
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
  static void (*const eqFunctions[18])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_26,
    UnderwaterDronePhysics_eqFunction_27,
    UnderwaterDronePhysics_eqFunction_28,
    UnderwaterDronePhysics_eqFunction_29,
    UnderwaterDronePhysics_eqFunction_30,
    UnderwaterDronePhysics_eqFunction_31,
    UnderwaterDronePhysics_eqFunction_32,
    UnderwaterDronePhysics_eqFunction_33,
    UnderwaterDronePhysics_eqFunction_34,
    UnderwaterDronePhysics_eqFunction_35,
    UnderwaterDronePhysics_eqFunction_36,
    UnderwaterDronePhysics_eqFunction_37,
    UnderwaterDronePhysics_eqFunction_38,
    UnderwaterDronePhysics_eqFunction_39,
    UnderwaterDronePhysics_eqFunction_40,
    UnderwaterDronePhysics_eqFunction_41,
    UnderwaterDronePhysics_eqFunction_42,
    UnderwaterDronePhysics_eqFunction_43
  };
  
  for (int id = 0; id < 18; id++) {
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
extern void UnderwaterDronePhysics_eqFunction_26(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_27(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_28(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_29(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_30(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_31(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_32(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_33(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_34(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_35(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_36(DATA* data, threadData_t *threadData);

static void functionODE_system0(DATA *data, threadData_t *threadData)
{
  static void (*const eqFunctions[11])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_26,
    UnderwaterDronePhysics_eqFunction_27,
    UnderwaterDronePhysics_eqFunction_28,
    UnderwaterDronePhysics_eqFunction_29,
    UnderwaterDronePhysics_eqFunction_30,
    UnderwaterDronePhysics_eqFunction_31,
    UnderwaterDronePhysics_eqFunction_32,
    UnderwaterDronePhysics_eqFunction_33,
    UnderwaterDronePhysics_eqFunction_34,
    UnderwaterDronePhysics_eqFunction_35,
    UnderwaterDronePhysics_eqFunction_36
  };
  
  if (data->simulationInfo->evalSelection) {
    for (int i = 0; i < data->simulationInfo->evalSelection->n; i++) {
      int id = data->simulationInfo->evalSelection->idx[i];
      eqFunctions[id](data, threadData);
    }
  } else {
    for (int id = 0; id < 11; id++) {
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
  const size_t eqMap[] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
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

#define _OMC_LIT_RESOURCE_1_name_data "Modelica"
#define _OMC_LIT_RESOURCE_1_dir_data "C:/Users/maxbu/AppData/Roaming/.openmodelica/libraries/Modelica 4.1.0+maint.om"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_1_name,8,_OMC_LIT_RESOURCE_1_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_1_dir,78,_OMC_LIT_RESOURCE_1_dir_data);

#define _OMC_LIT_RESOURCE_2_name_data "ModelicaServices"
#define _OMC_LIT_RESOURCE_2_dir_data "C:/Users/maxbu/AppData/Roaming/.openmodelica/libraries/ModelicaServices 4.1.0+maint.om"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_2_name,16,_OMC_LIT_RESOURCE_2_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_2_dir,86,_OMC_LIT_RESOURCE_2_dir_data);

#define _OMC_LIT_RESOURCE_3_name_data "UnderwaterDronePhysics"
#define _OMC_LIT_RESOURCE_3_dir_data "C:/Users/maxbu/Desktop/uni/VerificaEvalidazione/progetto/controllore"
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_3_name,22,_OMC_LIT_RESOURCE_3_name_data);
static const MMC_DEFSTRINGLIT(_OMC_LIT_RESOURCE_3_dir,68,_OMC_LIT_RESOURCE_3_dir_data);

static const MMC_DEFSTRUCTLIT(_OMC_LIT_RESOURCES,8,MMC_ARRAY_TAG) {MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_0_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_0_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_1_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_1_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_2_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_2_dir), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_3_name), MMC_REFSTRINGLIT(_OMC_LIT_RESOURCE_3_dir)}};
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
  data->modelData->modelDir = "C:/Users/maxbu/Desktop/uni/VerificaEvalidazione/progetto/controllore";
  data->modelData->modelGUID = "{3383d41d-3549-4780-812f-bc1ac2adce34}";
  data->modelData->initXMLData = NULL;
  data->modelData->modelDataXml.infoXMLData = NULL;
  GC_asprintf(&data->modelData->modelDataXml.fileName, "%s/UnderwaterDronePhysics_info.json", data->modelData->resourcesDir);
  data->modelData->runTestsuite = 0;
  data->modelData->nStatesArray = 7;
  data->modelData->nDiscreteReal = 0;
  data->modelData->nVariablesRealArray = 28;
  data->modelData->nVariablesIntegerArray = 0;
  data->modelData->nVariablesBooleanArray = 0;
  data->modelData->nVariablesStringArray = 0;
  data->modelData->nParametersRealArray = 10;
  data->modelData->nParametersIntegerArray = 0;
  data->modelData->nParametersBooleanArray = 0;
  data->modelData->nParametersStringArray = 0;
  data->modelData->nParametersReal = 10;
  data->modelData->nParametersInteger = 0;
  data->modelData->nParametersBoolean = 0;
  data->modelData->nParametersString = 0;
  data->modelData->nAliasRealArray = 0;
  data->modelData->nAliasIntegerArray = 0;
  data->modelData->nAliasBooleanArray = 0;
  data->modelData->nAliasStringArray = 0;
  data->modelData->nInputVars = 3;
  data->modelData->nOutputVars = 7;
  data->modelData->nZeroCrossings = 3;
  data->modelData->nSamples = 0;
  data->modelData->nRelations = 5;
  data->modelData->nMathEvents = 0;
  data->modelData->nExtObjs = 0;
  data->modelData->modelDataXml.modelInfoXmlLength = 0;
  data->modelData->modelDataXml.nFunctions = 0;
  data->modelData->modelDataXml.nProfileBlocks = 0;
  data->modelData->modelDataXml.nEquations = 48;
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

