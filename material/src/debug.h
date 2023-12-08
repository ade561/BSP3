/**
 * @file debug.h
 * @author Franz Korf, HAW Hamburg 
 * @date Nov 2014
 * @brief This header file defines some macros for debug output.
 */

#ifndef DEBUG_H 
#define DEBUG_H

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>

#ifdef DEBUG_MESSAGES
/**
 * Print debug message when DEBUG_MESSAGES is defined.
 */
#define PRINT_DEBUG(str) { \
	fprintf str; \
	fflush(stderr); \
}
#else
#define PRINT_DEBUG(str) { }
#endif /* DEBUG_MESSAGES */

#endif /* DEBUG_H */

