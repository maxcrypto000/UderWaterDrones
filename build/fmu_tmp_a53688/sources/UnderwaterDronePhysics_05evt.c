/* Events: Sample, Zero Crossings, Relations, Discrete Changes */
#include "UnderwaterDronePhysics_model.h"
#if defined(__cplusplus)
extern "C" {
#endif

/* Initializes the raw time events of the simulation using the now
   calcualted parameters. */
void UnderwaterDronePhysics_function_initSample(DATA *data, threadData_t *threadData)
{
  long i=0;
}

const char *UnderwaterDronePhysics_zeroCrossingDescription(int i, int **out_EquationIndexes)
{
  static const char *res[] = {"distance_to_base <= base_tolerance",
  "$outputAlias_battery >= battery_max and net_charge_rate > 0.0",
  "$outputAlias_battery <= 0.0 and net_charge_rate < 0.0"};
  static const int occurEqs0[] = {1,32};
  static const int occurEqs1[] = {1,33};
  static const int occurEqs2[] = {1,33};
  static const int *occurEqs[] = {occurEqs0,occurEqs1,occurEqs2};
  *out_EquationIndexes = (int*) occurEqs[i];
  return res[i];
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

int UnderwaterDronePhysics_function_ZeroCrossingsEquations(DATA *data, threadData_t *threadData)
{
  data->simulationInfo->callStatistics.functionZeroCrossingsEquations++;

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
  
  for (int id = 0; id < 11; id++) {
    eqFunctions[id](data, threadData);
  }
  
  return 0;
}

int UnderwaterDronePhysics_function_ZeroCrossings(DATA *data, threadData_t *threadData, double *gout)
{
  const int *equationIndexes = NULL;

  modelica_boolean tmp0;
  modelica_real tmp1;
  modelica_real tmp2;
  modelica_boolean tmp3;
  modelica_real tmp4;
  modelica_real tmp5;
  modelica_boolean tmp6;
  modelica_real tmp7;
  modelica_real tmp8;
  modelica_boolean tmp9;
  modelica_real tmp10;
  modelica_real tmp11;
  modelica_boolean tmp12;
  modelica_real tmp13;
  modelica_real tmp14;
  modelica_integer current_index = 0;
  modelica_integer start_index;
  
#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_tick(SIM_TIMER_ZC);
#endif
  data->simulationInfo->callStatistics.functionZeroCrossings++;

  start_index = current_index;
  tmp1 = 1.0;
  tmp2 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */));
  tmp0 = LessEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* distance_to_base variable */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */), tmp1, tmp2, data->simulationInfo->storedRelations[0]);
  gout[start_index] = (tmp0) ? 1 : -1;
  current_index++;

  start_index = current_index;
  tmp4 = 1.0;
  tmp5 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */));
  tmp3 = GreaterEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */), tmp4, tmp5, data->simulationInfo->storedRelations[1]);
  tmp7 = 1.0;
  tmp8 = 0.0;
  tmp6 = GreaterZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp7, tmp8, data->simulationInfo->storedRelations[2]);
  gout[start_index] = ((tmp3 && tmp6)) ? 1 : -1;
  current_index++;

  start_index = current_index;
  tmp10 = 1.0;
  tmp11 = 0.0;
  tmp9 = LessEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), 0.0, tmp10, tmp11, data->simulationInfo->storedRelations[3]);
  tmp13 = 1.0;
  tmp14 = 0.0;
  tmp12 = LessZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp13, tmp14, data->simulationInfo->storedRelations[4]);
  gout[start_index] = ((tmp9 && tmp12)) ? 1 : -1;
  current_index++;

#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_accumulate(SIM_TIMER_ZC);
#endif

  return 0;
}

const char *UnderwaterDronePhysics_relationDescription(int i)
{
  const char *res[] = {"distance_to_base <= base_tolerance",
  "$outputAlias_battery >= battery_max",
  "net_charge_rate > 0.0",
  "$outputAlias_battery <= 0.0",
  "net_charge_rate < 0.0"};
  return res[i];
}

int UnderwaterDronePhysics_function_updateRelations(DATA *data, threadData_t *threadData, int evalforZeroCross)
{
  const int *equationIndexes = NULL;

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
  modelica_real tmp29;
  modelica_integer current_index = 0;
  modelica_integer start_index;
  
  if(evalforZeroCross) {
    start_index = current_index;
    tmp16 = 1.0;
    tmp17 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */));
    tmp15 = LessEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* distance_to_base variable */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */), tmp16, tmp17, data->simulationInfo->storedRelations[0]);
    data->simulationInfo->relations[start_index] = tmp15;
    current_index++;

    start_index = current_index;
    tmp19 = 1.0;
    tmp20 = fabs((data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */));
    tmp18 = GreaterEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */), tmp19, tmp20, data->simulationInfo->storedRelations[1]);
    data->simulationInfo->relations[start_index] = tmp18;
    current_index++;

    start_index = current_index;
    tmp22 = 1.0;
    tmp23 = 0.0;
    tmp21 = GreaterZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp22, tmp23, data->simulationInfo->storedRelations[2]);
    data->simulationInfo->relations[start_index] = tmp21;
    current_index++;

    start_index = current_index;
    tmp25 = 1.0;
    tmp26 = 0.0;
    tmp24 = LessEqZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */), 0.0, tmp25, tmp26, data->simulationInfo->storedRelations[3]);
    data->simulationInfo->relations[start_index] = tmp24;
    current_index++;

    start_index = current_index;
    tmp28 = 1.0;
    tmp29 = 0.0;
    tmp27 = LessZC((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */), 0.0, tmp28, tmp29, data->simulationInfo->storedRelations[4]);
    data->simulationInfo->relations[start_index] = tmp27;
    current_index++;
  } else {
    start_index = current_index;
    data->simulationInfo->relations[start_index] = ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[15]] /* distance_to_base variable */) <= (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[0]] /* base_tolerance PARAM */));
    current_index++;

    start_index = current_index;
    data->simulationInfo->relations[start_index] = ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */) >= (data->simulationInfo->realParameter[data->simulationInfo->realParamsIndex[1]] /* battery_max PARAM */));
    current_index++;

    start_index = current_index;
    data->simulationInfo->relations[start_index] = ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */) > 0.0);
    current_index++;

    start_index = current_index;
    data->simulationInfo->relations[start_index] = ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[0]] /* $outputAlias_battery STATE(1) */) <= 0.0);
    current_index++;

    start_index = current_index;
    data->simulationInfo->relations[start_index] = ((data->localData[0]->realVars[data->simulationInfo->realVarsIndex[16]] /* net_charge_rate variable */) < 0.0);
    current_index++;
  }
  
  return 0;
}

#if defined(__cplusplus)
}
#endif
