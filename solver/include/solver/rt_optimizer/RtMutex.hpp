/*
 * Copyright [2019] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <pthread.h>

namespace rt_solver {

  struct RtMutex
  {
    public:
      typedef pthread_mutex_t MutexType;

      RtMutex() { init(); }
      ~RtMutex() { destroy(); }

      inline int lock() { return pthread_mutex_lock(&m_); }
      inline int unlock() { return pthread_mutex_unlock(&m_); }
      inline int trylock() { return pthread_mutex_trylock(&m_); }

      MutexType m_;

    private:
      inline int init() { return pthread_mutex_init(&m_, NULL); }
      inline int destroy() { return pthread_mutex_destroy(&m_); }
  };


  struct RtCond
  {
    public:
      typedef pthread_cond_t CondType;

      RtCond() { init(); }
      ~RtCond() { destroy(); }

      inline int signal() { return pthread_cond_signal(&c_); }
      inline int broadcast() { return pthread_cond_broadcast(&c_); }
      inline int wait(RtMutex& mutex) { return pthread_cond_wait(&c_, &mutex.m_); }

      CondType c_;

    private:
      inline int init() { return pthread_cond_init(&c_, NULL); }
      inline int destroy() { return pthread_cond_destroy(&c_); }
  };

}
