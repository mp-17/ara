// Copyright 2022 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#ifndef _SOFTMAX_H_
#define _SOFTMAX_H_

void softmax(const float *i, const float *o, const float *buf,
             uint64_t channels, uint64_t innerSize);

void softmax_vec(const float *i, const float *o, uint64_t channels,
                 uint64_t innerSize);

void softmax_vec_reduction(const double *i, const double *o, uint64_t channels,
                 uint64_t innerSize);

#endif
