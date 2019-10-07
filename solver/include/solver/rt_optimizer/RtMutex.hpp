/**
 * @file RtMutex.hpp
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
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
