// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// ElevationLayer 의 실측 높이로 지형을 읽는 TerrainSource.
//
// 왜 필요한가
// -----------
// CostmapTerrainSource 는 master costmap 의 cost 값을 높이 등급으로 읽는다.
// 그런데 master 는 레이어들의 '최댓값'이다:
//   - obstacle_layer 가 LiDAR 로 본 장애물을 254 로 찍으면, ElevationLayer 가
//     같은 셀을 200(fly-over)으로 분류해도 덮어쓰지 못한다 -> 날아 넘을 장애물이
//     통과 불가가 되어 비행 경로 자체가 안 나온다.
//   - inflation_layer 의 그라디언트(1~252)가 높이 등급으로 오독된다 ->
//     충분히 주행 가능한 좁은 통로가 '비행 필요'로 바뀐다.
//   - fly-over 등급(0.15~1.2m)을 대표값 하나로 읽으므로 1.0m 박스를 0.6m 로
//     보고 0.8m 로 뚫고 지나가는 경로가 나온다.
//
// 그래서 높이는 ElevationLayer 에서 직접 읽고, master costmap 은
// '로버가 설 수 있는가'(footprint 충돌, 254/253)와 '높이 모를 장애물'에만 쓴다.

#ifndef DROBOT_HYBRID_PLANNER__ELEVATION_TERRAIN_SOURCE_HPP_
#define DROBOT_HYBRID_PLANNER__ELEVATION_TERRAIN_SOURCE_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <drobot_costmap_2_5d/elevation_layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "drobot_hybrid_planner/state_space.hpp"

namespace drobot_hybrid_planner
{

class ElevationTerrainSource : public TerrainSource
{
public:
  ElevationTerrainSource(
    nav2_costmap_2d::Costmap2D * master,
    std::shared_ptr<drobot_costmap_2_5d::ElevationLayer> layer,
    const CostmapTerrainSource::Config & fallback_cfg)
  : master_(master), layer_(std::move(layer)), cfg_(fallback_cfg),
    fallback_(master, fallback_cfg) {}

  /// 경로계획 시작 시 높이 스냅샷을 받는다 (호출자는 master costmap 잠금을 잡고 있다).
  void update() override
  {
    layer_->snapshotTerrain(height_, cls_);
    const size_t n =
      static_cast<size_t>(master_->getSizeInCellsX()) * master_->getSizeInCellsY();
    valid_ = (height_.size() == n && cls_.size() == n);
    cost_rover_ = layer_->costRover();
    cost_impassable_ = layer_->costImpassable();
  }

  double heightAt(unsigned int mx, unsigned int my) const override
  {
    if (!valid_) {return fallback_.heightAt(mx, my);}
    const size_t k = index(mx, my);
    const unsigned char cls = cls_[k];
    // 천장 제약 등으로 ElevationLayer 가 이미 통과 불가로 판정한 셀
    if (cls == cost_impassable_) {return cfg_.h_impassable;}
    // RGB-D 로 높이를 잰 셀 — 이 높이만 믿고 날아 넘는다
    if (!std::isnan(height_[k])) {return std::max(0.0, static_cast<double>(height_[k]));}
    // 높이를 모르는 장애물: LiDAR 만 본 셀(이 레이어가 rover 이상으로 분류)이나
    // 다른 레이어의 lethal. 실제 높이를 모르니 넘을 수 없다고 본다.
    if (cls != nav2_costmap_2d::NO_INFORMATION && cls > cost_rover_) {return cfg_.h_impassable;}
    if (master_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {return cfg_.h_impassable;}
    // 나머지(평지, inflation 그라디언트)는 지면이다
    return cfg_.h_free;
  }

  bool roverTraversable(unsigned int mx, unsigned int my) const override
  {
    if (!valid_) {return fallback_.roverTraversable(mx, my);}
    const unsigned char c = master_->getCost(mx, my);
    if (c == nav2_costmap_2d::NO_INFORMATION) {return false;}
    // footprint 충돌 (inflation 의 inscribed 이상)
    if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
    // 높이/거칠기 때문에 주행 불가로 분류된 셀
    const unsigned char cls = cls_[index(mx, my)];
    if (cls != nav2_costmap_2d::NO_INFORMATION && cls > cost_rover_) {return false;}
    // inflation 그라디언트(1~252)는 '가까울수록 비싸다'는 뜻이지 장애물이 아니다
    return true;
  }

private:
  size_t index(unsigned int mx, unsigned int my) const
  {
    return static_cast<size_t>(my) * master_->getSizeInCellsX() + mx;
  }

  nav2_costmap_2d::Costmap2D * master_;
  std::shared_ptr<drobot_costmap_2_5d::ElevationLayer> layer_;
  CostmapTerrainSource::Config cfg_;
  CostmapTerrainSource fallback_;

  std::vector<float> height_;
  std::vector<unsigned char> cls_;
  bool valid_ = false;
  unsigned char cost_rover_ = 100;
  unsigned char cost_impassable_ = 254;
};

}  // namespace drobot_hybrid_planner

#endif  // DROBOT_HYBRID_PLANNER__ELEVATION_TERRAIN_SOURCE_HPP_
