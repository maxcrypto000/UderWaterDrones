/* Algebraic */
#include "UnderwaterDronePhysics_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* forwarded equations */
extern void UnderwaterDronePhysics_eqFunction_37(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_38(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_39(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_40(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_41(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_42(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_43(DATA* data, threadData_t *threadData);

static void functionAlg_system0(DATA *data, threadData_t *threadData)
{
  static void (*const eqFunctions[7])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_37,
    UnderwaterDronePhysics_eqFunction_38,
    UnderwaterDronePhysics_eqFunction_39,
    UnderwaterDronePhysics_eqFunction_40,
    UnderwaterDronePhysics_eqFunction_41,
    UnderwaterDronePhysics_eqFunction_42,
    UnderwaterDronePhysics_eqFunction_43
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
/* for continuous time variables */
int UnderwaterDronePhysics_functionAlgebraics(DATA *data, threadData_t *threadData)
{

#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_tick(SIM_TIMER_ALGEBRAICS);
#endif
  data->simulationInfo->callStatistics.functionAlgebraics++;

  UnderwaterDronePhysics_function_savePreSynchronous(data, threadData);
  
  functionAlg_system0(data, threadData);

#if !defined(OMC_MINIMAL_RUNTIME)
  if (measure_time_flag) rt_accumulate(SIM_TIMER_ALGEBRAICS);
#endif

  return 0;
}

#ifdef __cplusplus
}
#endif
