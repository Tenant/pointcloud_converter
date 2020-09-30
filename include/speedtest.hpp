//
// Created by akira on 19-2-25.
//

#include <time.h>

#ifndef SYSOUT_F
#define SYSOUT_F(f, ...)      printf( f, __VA_ARGS__ )
#endif

#ifndef speedtest__
#define speedtest__(data)   for (long blockTime = NULL; (blockTime == NULL ? (blockTime = clock()) != NULL : false); SYSOUT_F(data "%.9fs\n", (double) (clock() - blockTime) / CLOCKS_PER_SEC))
#endif

