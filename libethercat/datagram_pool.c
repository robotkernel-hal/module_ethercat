//! ethercat datagram pool
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of libethercat.
 *
 * libethercat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libethercat is distributed in the hope that 
 * it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libethercat
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "datagram_pool.h"
#include <errno.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>


//! open a new datagram pool
/*!
 * \param pp return datagram_pool 
 * \param cnt number of packets in pool
 * \return 0 or negative error code
 */
int datagram_pool_open(datagram_pool_t **pp, size_t cnt) {
    (*pp) = (datagram_pool_t *)malloc(sizeof(datagram_pool_t));
    if (!(*pp))
        return -ENOMEM;

    pthread_mutex_init(&(*pp)->_pool_lock, NULL);
    pthread_mutex_lock(&(*pp)->_pool_lock);

    memset(&(*pp)->avail_cnt, 0, sizeof(sem_t));
    sem_init(&(*pp)->avail_cnt, 0, cnt);
    TAILQ_INIT(&(*pp)->avail);

    int i;
    for (i = 0; i < cnt; ++i) {
        datagram_entry_t *datagram = 
            (datagram_entry_t *)malloc(sizeof(datagram_entry_t) + 1500);
        memset(datagram, 0, sizeof(datagram_entry_t) + 1500);
        TAILQ_INSERT_TAIL(&(*pp)->avail, datagram, qh);
    }
    
    pthread_mutex_unlock(&(*pp)->_pool_lock);

    return 0;
}

//! destroys a datagram pool
/*!
 * \param pp datagram_pool handle
 * \return 0 or negative error code
 */
int datagram_pool_close(datagram_pool_t *pp) {
    if (!pp)
        return -EINVAL;
    
    pthread_mutex_lock(&pp->_pool_lock);

    datagram_entry_t *datagram;
    while ((datagram = TAILQ_FIRST(&pp->avail)) != NULL) {
        TAILQ_REMOVE(&pp->avail, datagram, qh);
        free(datagram);
    }
    
    pthread_mutex_unlock(&pp->_pool_lock);
    pthread_mutex_destroy(&pp->_pool_lock);

    free(pp);
    
    return 0;
}

# define timespecadd(a, b, result)                                            \
  do {                                                                        \
    (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;                             \
    (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec;                          \
    if ((result)->tv_nsec >= 1E9)                                             \
      {                                                                       \
        ++(result)->tv_sec;                                                   \
        (result)->tv_nsec -= 1E9;                                             \
      }                                                                       \
  } while (0)

//! get a datagram from datagram_pool
/*!
 * \param pp datagram_pool handle
 * \param datagram ec datagram pointer
 * \param ts timeout waiting for packet
 * \return 0 or negative error code
 */
int datagram_pool_get(datagram_pool_t *pp, datagram_entry_t **datagram, struct timespec *ts) {
    int ret = ENOPKG;
    if (!pp || !datagram)
        return (ret = EINVAL);

    if (ts) {
        struct timespec act, end;
        ret = clock_gettime(CLOCK_REALTIME, &act);
        if (ret != 0)
            perror("clock_gettime");

        timespecadd(&act, ts, &end);

        ret = sem_timedwait(&pp->avail_cnt, &end);
        if (ret != 0) {
            if (errno != ETIMEDOUT)
                perror("sem_timedwait");

            return (ret = errno);
        }
    }

    pthread_mutex_lock(&pp->_pool_lock);

    *datagram = (datagram_entry_t *)TAILQ_FIRST(&pp->avail);
    if (*datagram) {
        TAILQ_REMOVE(&pp->avail, (datagram_entry_t *)*datagram, qh);
        ret = 0;
    }
    
    pthread_mutex_unlock(&pp->_pool_lock);

    return ret;
}

//! get next datagram length from datagram_pool
/*!
 * \param pp datagram_pool handle
 * \param len datagram length
 * \return 0 or negative error code
 */
int datagram_pool_get_next_len(datagram_pool_t *pp, size_t *len) {
    if (!pp || !len)
        return EINVAL;
    

    pthread_mutex_lock(&pp->_pool_lock);

    datagram_entry_t *entry = (datagram_entry_t *)TAILQ_FIRST(&pp->avail);
    if (entry)
        *len = ec_datagram_length(&entry->datagram);
    else
        *len = 0;
    
    pthread_mutex_unlock(&pp->_pool_lock);

    return ENOPKG;
}

//! return a datagram to datagram_pool
/*!
 * \param pp datagram_pool handle
 * \param datagram ec datagram pointer
 * \return 0 or negative error code
 */
int datagram_pool_put(datagram_pool_t *pp, datagram_entry_t *datagram) {
    if (!pp || !datagram)
        return -EINVAL;
    
    pthread_mutex_lock(&pp->_pool_lock);

    TAILQ_INSERT_TAIL(&pp->avail, (datagram_entry_t *)datagram, qh);
    sem_post(&pp->avail_cnt);
    
    pthread_mutex_unlock(&pp->_pool_lock);
    
    return 0;
}

