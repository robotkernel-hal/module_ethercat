#ifndef __TIMER_H__
#define __TIMER_H__

#define NSEC_PER_SEC                1000000000

#define EC_DEFAULT_TIMEOUT_MBX      100000000
#define EC_DEFAULT_DELAY            2000000

#include <stdint.h>
#include <time.h>
#include <sys/time.h>

typedef struct ec_timer {
    uint64_t sec;
    uint64_t nsec;
} ec_timer_t;

//! sleep in nanoseconds
/*!
 * \param nsec time to sleep in nanoseconds
 */
void ec_sleep(uint64_t nsec);

//! initialize timer with timeout 
/*!
 * \parma timer pointer to timer to initialize
 * \param timeout in nanoseconds
 */
void ec_timer_init(ec_timer_t *timer, uint64_t timeout);

//! checks if timer is expired
/*!
 * \param timer timer to check 
 * \return 1 if expired, 0 if not
 */
int ec_timer_expired(ec_timer_t *timer);

#endif // __TIMER_H__

