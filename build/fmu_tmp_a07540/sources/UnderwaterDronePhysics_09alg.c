/* Algebraic */
#include "UnderwaterDronePhysics_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* forwarded equations */
extern void UnderwaterDronePhysics_eqFunction_27(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_28(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_29(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_30(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_31(DATA* data, threadData_t *threadData);
extern void UnderwaterDronePhysics_eqFunction_32(DATA* data, threadData_t *threadData);

static void functionAlg_system0(DATA *data, threadData_t *threadData)
{
  static void (*const eqFunctions[6])(DATA*, threadData_t*) = {
    UnderwaterDronePhysics_eqFunction_27,
    UnderwaterDronePhysics_eqFunction_28,
    UnderwaterDronePhysics_eqFunction_29,
    UnderwaterDronePhysics_eqFunction_30,
    UnderwaterDronePhysics_eqFunction_31,
    UnderwaterDronePhysics_eqFunction_32
  };
  
  if (data->simulationInfo->evalSelection) {
    for (int i = 0; i < data->simulationInfo->evalSelection->n; i++) {
      int id = data->simulationInfo->evalSelection->idx[i];
      eqFunctions[id](data, threadData);
    }
  } else {
    for (int id = 0; id < 6; id++) {
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
