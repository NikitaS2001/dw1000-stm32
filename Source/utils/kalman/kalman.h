#pragma once

#define   LENGTH      1*1
#define   ORDER       1
#define   N           100
#define   SEED        1567

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
} tOptimal;

extern void   KalMan_PramInit(void);
extern float KalMan_Update(double *Z);

#ifdef __cplusplus
}
#endif
