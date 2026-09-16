#if defined(__cplusplus)
  extern "C" {
#endif
  int UnderwaterDronePhysics_mayer(DATA* data, modelica_real** res, short*);
  int UnderwaterDronePhysics_lagrange(DATA* data, modelica_real** res, short *, short *);
  int UnderwaterDronePhysics_getInputVarIndicesInOptimization(DATA* data, int* input_var_indices);
  int UnderwaterDronePhysics_pickUpBoundsForInputsInOptimization(DATA* data, modelica_real* min, modelica_real* max, modelica_real*nominal, modelica_boolean *useNominal, char ** name, modelica_real * start, modelica_real * startTimeOpt);
  int UnderwaterDronePhysics_setInputData(DATA *data, const modelica_boolean file);
  int UnderwaterDronePhysics_getTimeGrid(DATA *data, modelica_integer * nsi, modelica_real**t);
#if defined(__cplusplus)
}
#endif
