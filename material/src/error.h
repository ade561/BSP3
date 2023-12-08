/**
 * @file error.h
 * @author Franz Korf, HAW Hamburg 
 * @date Nov 2014
 * @brief This header file defines some macros for error handling and output.
 */

#ifndef ERROR_H 
#define ERROR_H

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>

/**
 * Based on a condition this macro  generates an perror message an exits.
 */
#define TEST_AND_EXIT_ERRNO(cond, str) if (cond) { \
        perror(str); \
        exit(EXIT_FAILURE); \
}

/**
 * Based on a condition this macro prints an error message to stderr and exits.
 */
#define TEST_AND_EXIT(cond, str) if (cond) { \
        fprintf str; \
        exit(EXIT_FAILURE); \
}

#endif /* ERROR_H */

