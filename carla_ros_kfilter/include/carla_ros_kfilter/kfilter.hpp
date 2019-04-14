//
// Created by pedro on 14-04-2019.
//

// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           �cole Polytechnique de Montr�al
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef ROS_BRIDGE_KFILTER_HPP
#define ROS_BRIDGE_KFILTER_HPP

//! \file
//! \brief Contains the interface of the \c KFilter base template class.

// System Includes
#include <carla_ros_kfilter/ekfilter.hpp>

namespace Kalman {
    // TODO : il faut que E(v) == 0 && E(w) == 0 !!!
    template<typename T, K_UINT_32 BEG, bool OQ = false, bool OVR = false, bool DBG = true>

    class KFilter : public EKFilter<T, BEG, OQ, OVR, DBG>{
        public:
            virtual ~KFilter() = 0;
        protected:
            virtual void makeBaseB();
            virtual void makeB();
            Matrix B;
        private:
            virtual void makeProcess();
            virtual void makeMeasure();
            virtual void sizeUpdate();
            Vector x__;
    };
}

#include <carla_ros_kfilter/kfilter_impl.hpp>


#endif //ROS_BRIDGE_KFILTER_HPP
