/* Jacobians */
static _index_t one_dim[1] = { 1 };
static modelica_real nominal_data[1] = { 1.0 };
static modelica_real start_data[1]   = { 0.0 };
static modelica_real min_data[1]   = { -DBL_MAX };
static modelica_real max_data[1]   = { DBL_MAX };
static const REAL_ATTRIBUTE dummyREAL_ATTRIBUTE = {
  .unit = NULL,
  .displayUnit = NULL,
  .min = {
    .ndims     = 1,
    .dim_size  = one_dim,
    .data      = (void*) min_data,
    .flexible  = FALSE
  },
  .max = {
    .ndims     = 1,
    .dim_size  = one_dim,
    .data      = (void*) max_data,
    .flexible  = FALSE
  },
  .fixed = FALSE,
  .useNominal = FALSE,
  .nominal = {
    .ndims     = 1,
    .dim_size  = one_dim,
    .data      = (void*) nominal_data,
    .flexible  = FALSE
  },
  .start = {
    .ndims     = 1,
    .dim_size  = one_dim,
    .data      = (void*) start_data,
    .flexible  = FALSE
  }
};

#if defined(__cplusplus)
extern "C" {
#endif

/* Jacobian Variables */
#define UnderwaterDronePhysics_INDEX_JAC_ADJ 0
int UnderwaterDronePhysics_functionJacADJ_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianADJ(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacADJ_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_H 1
int UnderwaterDronePhysics_functionJacH_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianH(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacH_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_F 2
int UnderwaterDronePhysics_functionJacF_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianF(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacF_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_D 3
int UnderwaterDronePhysics_functionJacD_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianD(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacD_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_C 4
int UnderwaterDronePhysics_functionJacC_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianC(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacC_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_B 5
int UnderwaterDronePhysics_functionJacB_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianB(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacB_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);


#define UnderwaterDronePhysics_INDEX_JAC_A 6
int UnderwaterDronePhysics_functionJacA_column(DATA* data, threadData_t *threadData, JACOBIAN *thisJacobian, JACOBIAN *parentJacobian);
int UnderwaterDronePhysics_initialAnalyticJacobianA(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);
void UnderwaterDronePhysics_JacA_DAG(DATA* data, threadData_t *threadData, JACOBIAN *jacobian);

#if defined(__cplusplus)
}
#endif
