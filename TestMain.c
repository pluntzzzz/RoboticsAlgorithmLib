/**
 * @brief			Description: Run the test function in main function.
 * @file:			TestMain.c
 * @author:			Brian
 * @date:			2019/03/01 12:23
 * Copyright(c) 	2019 Brian. All rights reserved.
 *
 * Contact 			https://blog.csdn.net/Galaxy_Robot
 * @note:     
 * @warning: 		
*/
#include "TestDemo.h"
#include <sys/time.h>
#include <stdio.h>
int main()
{
    struct timeval stop, start;
    gettimeofday(&start, NULL);

    //do stuff

    test_MatrixPinv67();

    //end stuff

    gettimeofday(&stop, NULL);
    printf("took %lu us\n", (stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec);

    return 0;
}