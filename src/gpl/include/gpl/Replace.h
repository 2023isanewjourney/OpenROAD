///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2018-2020, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <vector>

namespace odb {
class dbDatabase;
class dbInst;
}  // namespace odb
namespace sta {
class dbSta;
}

namespace grt {
class GlobalRouter;
}

namespace rsz {
class Resizer;
}

namespace utl {
class Logger;
}

namespace gpl {

class PlacerBase;
class NesterovBase;
class RouteBase;
class TimingBase;

class InitialPlace;
class NesterovPlace;
class Debug;

class Replace
{
 public:
  Replace();
  ~Replace();

  /**
   * @fn void init(odb::dbDatabase*, rsz::Resizer*, grt::GlobalRouter*, utl::Logger*)
   * @brief initializes a Replace with a Openroad database, a Global router, and a Gate resizer.
   *
   * @pre
   * @post
   * @param odb
   * @param resizer size gates in a post synthesis involveing buffer ports, repairing cap
   * and slew violations, resizing gates to reduce slew variation (https://github.com/The-OpenROAD-Project-Attic/Resizer)
   * @param router FastRoute is an open-source global router
   * @param logger
   */
  void init(odb::dbDatabase* odb,
            rsz::Resizer* resizer,
            grt::GlobalRouter* router,
            utl::Logger* logger);
  void reset();

  void doIncrementalPlace();

  /**
   * @fn void doInitialPlace()
   * @brief runs a Bi-Conjugate gradient stabilized placement on CPU or GPU which is configurable.
   * It creates an InitialPlace object. Then, to process the object,
   * under the hood, it runs a series of functions:
   *  - set center location for instances;
   *  - set Ext Id and reset Pin Min Max attributes
   *  - finally, it creates Sparse  Metrixs from a PlacerBaseCommon object, which supposes to include everything from PlacerBase that is not region specific.
   * Then, BiCGSTAB Eigen Solver is used to compute the Eigen Vectors for X and Y dimensions.
   *
   * @pre
   * @post
   * @note InitialPlace::createSparseMatrix, gpl::InitialPlace, InitialPlace::createSparseMatrix()
   */
  void doInitialPlace();

  /**
   * @fn int doNesterovPlace(int=0)
   * @brief calls Nestero Placer to run placement
   *
   * @pre initNesterovplace is required
   * @post
   * @param start_iter
   * @return
   */
  int doNesterovPlace(int start_iter = 0);

  // Initial Place param settings
  void setInitialPlaceMaxIter(int iter);
  /**
   * @fn void setInitialPlaceMinDiffLength(int)
   * @brief sets minDiffLength used for B2B modeling while creating InitialPlace::createSparseMatrix
   * The variable is used in createSparseMatrix during Initial Placement.
   *
   * @pre
   * @post
   * @param length
   */
  void setInitialPlaceMinDiffLength(int length);
  void setInitialPlaceMaxSolverIter(int iter);
  /**
     * @fn void setInitialPlaceMaxFanout(int)
   * @brief set ipVars_.maxFanout for B2B modeling to escape long time cals on huge fanout.
   *  The variable is used in createSparseMatrix during Initial Placement.
   * @pre
   * @post
   * @param fanout
   */
  void setInitialPlaceMaxFanout(int fanout);

  /**
     * @fn void setInitialPlaceNetWeightScale(float)
   * @brief set net weight to each net, used in createSparseMatrix during Initial Placement.
   *
   * @pre
   * @post
   * @param scale
   */
  void setInitialPlaceNetWeightScale(float scale);

  void setNesterovPlaceMaxIter(int iter);

  void setBinGridCnt(int binGridCntX, int binGridCntY);

  void setTargetDensity(float density);
  void setUniformTargetDensityMode(bool mode);
  void setTargetOverflow(float overflow);
  void setInitDensityPenalityFactor(float penaltyFactor);
  void setInitWireLengthCoef(float coef);
  void setMinPhiCoef(float minPhiCoef);
  void setMaxPhiCoef(float maxPhiCoef);

  float getUniformTargetDensity();

  // HPWL: half-parameter wire length.
  void setReferenceHpwl(float deltaHpwl);

  // temp funcs; OpenDB should have these values.
  void setPadLeft(int padding);
  void setPadRight(int padding);

  void setForceCPU(bool force_cpu);
  void setTimingDrivenMode(bool mode);

  void setSkipIoMode(bool mode);

  void setRoutabilityDrivenMode(bool mode);
  void setRoutabilityCheckOverflow(float overflow);
  void setRoutabilityMaxDensity(float density);

  void setRoutabilityMaxBloatIter(int iter);
  void setRoutabilityMaxInflationIter(int iter);

  void setRoutabilityTargetRcMetric(float rc);
  void setRoutabilityInflationRatioCoef(float ratio);
  void setRoutabilityMaxInflationRatio(float ratio);

  void setRoutabilityRcCoefficients(float k1, float k2, float k3, float k4);

  void addTimingNetWeightOverflow(int overflow);
  void setTimingNetWeightMax(float max);

  void setDebug(int pause_iterations,
                int update_iterations,
                bool draw_bins,
                bool initial,
                odb::dbInst* inst = nullptr);

 private:
  bool initNesterovPlace();

  odb::dbDatabase* db_;
  rsz::Resizer* rs_;
  grt::GlobalRouter* fr_;
  utl::Logger* log_;

  std::shared_ptr<PlacerBase> pb_;
  std::shared_ptr<NesterovBase> nb_;
  std::shared_ptr<RouteBase> rb_;
  std::shared_ptr<TimingBase> tb_;

  std::unique_ptr<InitialPlace> ip_;
  std::unique_ptr<NesterovPlace> np_;

  // Members for Initial placement

  int initialPlaceMaxIter_;
  /// parameter used for B2B modeling
  int initialPlaceMinDiffLength_;
  int initialPlaceMaxSolverIter_;
  int initialPlaceMaxFanout_;
  float initialPlaceNetWeightScale_;
  bool forceCPU_;

  // Members to initialize np variable for Nesterov Placement
  int nesterovPlaceMaxIter_;
  int binGridCntX_;
  int binGridCntY_;
  float overflow_;
  float density_;
  float initDensityPenalityFactor_;
  float initWireLengthCoef_;
  float minPhiCoef_;
  float maxPhiCoef_;
  float referenceHpwl_;

  float routabilityCheckOverflow_;
  float routabilityMaxDensity_;
  float routabilityTargetRcMetric_;
  float routabilityInflationRatioCoef_;
  float routabilityMaxInflationRatio_;

  // routability RC metric coefficients
  float routabilityRcK1_, routabilityRcK2_, routabilityRcK3_, routabilityRcK4_;

  int routabilityMaxBloatIter_;
  int routabilityMaxInflationIter_;

  float timingNetWeightMax_;

  bool timingDrivenMode_;
  bool routabilityDrivenMode_;
  bool uniformTargetDensityMode_;
  bool skipIoMode_;

  std::vector<int> timingNetWeightOverflows_;

  // temp variable; OpenDB should have these values.
  int padLeft_;
  int padRight_;

  bool gui_debug_;
  int gui_debug_pause_iterations_;
  int gui_debug_update_iterations_;
  int gui_debug_draw_bins_;
  int gui_debug_initial_;
  odb::dbInst* gui_debug_inst_;
};
}  // namespace gpl
