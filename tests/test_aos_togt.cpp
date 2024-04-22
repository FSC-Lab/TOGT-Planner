#include <gtest/gtest.h>
#include "drolib/race/race_track.hpp"
#include "drolib/race/race_params.hpp"
#include "drolib/race/race_planner.hpp"
#include <filesystem>
using namespace drolib;

class AosTogtTest : public ::testing::Test {
 protected:
  std::shared_ptr<RaceTrack> racetrack;
  std::shared_ptr<RacePlanner> raceplanner;
  std::shared_ptr<RaceParams> raceparams;
  const std::string quad_name = "cpc";
  const std::string config_name = quad_name + "_setups.yaml";
  const std::string track_name = "race_uzh_19g.yaml";
  const std::string traj_name ="aos_togt_traj.csv";
  const std::string wpt_name ="aos_togt_wpt.yaml";

};

TEST_F(AosTogtTest, planTraj) {
  fs::path cwd = std::filesystem::current_path();
  fs::path config_path = cwd / ".." / "parameters" / quad_name;
  fs::path track_path = cwd / ".." / "resources/racetrack" / track_name;
  fs::path traj_path = cwd / ".." / "resources/trajectory" / traj_name;
  fs::path wpt_path = cwd / ".." / "resources/trajectory" / wpt_name;

  raceparams = std::make_shared<RaceParams>(config_path, config_name);
  raceplanner = std::make_shared<RacePlanner>(*raceparams);

  racetrack = std::make_shared<RaceTrack>(track_path);
  EXPECT_TRUE(raceplanner->plan(racetrack));

  TrajExtremum extremum = raceplanner->getExtremum();
  std::cout << extremum << std::endl;

  MincoSnapTrajectory traj = raceplanner->getTrajectory();
  traj.save(traj_path);  
  traj.saveSegments(wpt_path, raceparams->tprefine.piecesPerSegment);
}

