#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, const bool use_formation)
  { 
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);
    Eigen::Vector3d start_pos = iniState.col(0);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ;
    bool use_formation_temp = use_formation_;

    double q[variable_num_];
    memcpy(q, initInnerPts.data(), initInnerPts.size() * sizeof(q[0]));
    Eigen::Map<Eigen::VectorXd> Vt(q + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.g_epsilon = 0.1;     //0.1
    lbfgs_params.min_step = 1e-32;    //1e-32
    
    /* trick : for real-time optimization */
    if(use_formation){
      // consider formation
      // so we use less iterations for real time optimization
      lbfgs_params.max_iterations = 20; //200      
    } else{
      // do not consider formation
      // so we use more iterations for more precise optimization results
      lbfgs_params.max_iterations = 60; //200
      use_formation_ = false;
    }

    iter_num_ = 0;
    flag_force_return = false;
    force_stop_type_ = DONT_STOP;
    flag_still_occ = false;
    /* ---------- optimize ---------- */

    t1 = ros::Time::now();
    int result = lbfgs::lbfgs_optimize(
        variable_num_,
        q,
        &final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        &lbfgs_params);
    
    // test collision
    bool occ = false;
    occ = checkCollision();
    
    use_formation_ = use_formation_temp;

    t2 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;
    double total_time_ms = (t2 - t0).toSec() * 1000;

    printf("\033[32miter=%d,time(ms)=%5.3f, \n\033[0m", iter_num_, time_ms);
    // ROS_WARN("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    optimal_points = cps_.points;

    showFormationInformation(false, start_pos);
    
    if (occ)
      return false;
    else
      return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  std::vector<std::pair<int, int>> PolyTrajOptimizer::setAndFinelyCheckConstrainPoints(
      Eigen::MatrixXd &init_points,
      bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - 1 - ((int)init_points.cols() - 2) / 3; // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {
      for (double a = 1.0; a > 0.0; a -= step_size)
      {
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 1)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    // return in advance
    if (segment_ids.size() == 0)
    {
      vector<std::pair<int, int>> blank_ret;
      return blank_ret;
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out, false))
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (i + 1 < segment_ids.size()) // connect the next segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A conor case 2, I have never exeam it.");
      }
      else
      {
        ROS_ERROR("A-star error, force return!");
        vector<std::pair<int, int>> blank_ret;
        return blank_ret;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return final_segment_ids;
  }

  bool PolyTrajOptimizer::checkCollision(void){
    // only check the 2/3 of trajectory
    bool occ = false;
    double dt = 0.01;
    poly_traj::Trajectory traj = jerkOpt_.getTraj();
    double T_all = traj.getTotalDuration();
    int i_end = round(T_all/dt) * 3 / 5;
    double t = 0.0;

    for (int i=0; i<i_end; i++){
      Eigen::Vector3d pos = traj.getPos(t);
      occ = grid_map_->getInflateOccupancy(pos);
      t += dt;
      if (occ)
        break;
    }
    return occ;
  }

  bool PolyTrajOptimizer::roughlyCheckConstrainPoints(void)
  {

    int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - 1) / 3;
    for (int i = 1; i < i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out, false))
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (i + 1 < segment_ids.size()) // connect the next segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A conor case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR("A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  /* multi-topo support */
  std::vector<ConstrainPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstrainPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstrainPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstrainPoints, ConstrainPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstrainPoints, ConstrainPoints> RichInfoOneSeg;
      ConstrainPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstrainPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN("Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN("base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            //cout << base_pt_temp.transpose() << endl;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN("Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN("base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstrainPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // cout << "A4" << endl;

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstrainPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  } // namespace ego_planner

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  { 
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(6);

    opt->jerkOpt_.generate(P, T);

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    // if (opt->iter_num_ > 3 && smoo_cost / opt->piece_num_ < 10.0) // 10.0 is an experimental value that indicates the trajectory is smooth enough.
    // {
    //   opt->roughlyCheckConstrainPoints();
    // }
    
    opt->jerkOpt_.getGrad2TP(gradT, gradP);
    // time_cost += opt->rho_ * T(0) * piece_nums;  // same t
    // grad[n - 1] = (gradT.sum() + opt->rho_ * piece_nums) * gdT2t(x[n - 1]);  // same t

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    // debug
    // if (opt->drone_id_ == 0){
    //   double cost_sum = obs_swarm_feas_qvar_costs.sum();
    //   cout << "----iter_num_ : " << opt->iter_num_ << "  ----cost sum : " << cost_sum << " ----"<< endl;
    //   cout << "collision cost           : " << obs_swarm_feas_qvar_costs(0) << " , " << 100*obs_swarm_feas_qvar_costs(0)/cost_sum << "%" << endl;
    //   cout << "swarm collision cost     : " << obs_swarm_feas_qvar_costs(1) << " , " << 100*obs_swarm_feas_qvar_costs(1)/cost_sum << "%" << endl;
    //   cout << "swarm formation cost     : " << obs_swarm_feas_qvar_costs(2) << " , " << 100*obs_swarm_feas_qvar_costs(2)/cost_sum << "%" << endl;
    //   cout << "formation gathering cost : " << obs_swarm_feas_qvar_costs(3) << " , " << 100*obs_swarm_feas_qvar_costs(3)/cost_sum << "%" << endl;
    //   cout << "dynamic fisibility cost  : " << obs_swarm_feas_qvar_costs(4) << " , " << 100*obs_swarm_feas_qvar_costs(4)/cost_sum << "%" << endl;
    //   cout << "quratic variance cost    : " << obs_swarm_feas_qvar_costs(5) << " , " << 100*obs_swarm_feas_qvar_costs(5)/cost_sum << "%" << endl;
    // }

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    //
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradp, gradv, grada;
    double costp, costv, costa;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    double t = 0;
    // Eigen::MatrixXd constrain_pts(3, N * K + 1);
    
    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        /* Formation method chosing */
        if (use_formation_){
          switch (formation_method_type_)
          {
            case FORMATION_METHOD_TYPE::SWARM_GRAPH:
            {
              // deformale formation
              if (swarmGraphGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)){
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0)
                {
                  gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(2) += omg * step * costp;
              }
              
              // swarm gathering
              if (swarmGatherCostGradP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)){
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0)
                {
                  gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(3) += omg * step * costp;
              }
              break;
            }

            case FORMATION_METHOD_TYPE::LEADER_POSITION :
            {
              // benchmark: leader-follower position-based formation
              if (leaderPosFormationCostGradP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)){
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0)
                {
                  gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(3) += omg * step * costp;
              }
              break;
            }
            
            case FORMATION_METHOD_TYPE::RELATIVE_POSITION :
            {
              // benchmark: relative position-based formation
              if (relativePosFormationCostGradP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp)){
                gradViolaPc = beta0 * gradp.transpose();
                gradViolaPt = alpha * gradt;
                jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
                gdT(i) += omg * (costp / K + step * gradViolaPt);
                if (i > 0)
                {
                  gdT.head(i).array() += omg * step * grad_prev_t;
                }
                costs(3) += omg * step * costp;
              }
              break;
            }

            case FORMATION_METHOD_TYPE::VRB_METHOD :
            {
              // benchmark: VRB-method 

            }
            
            default:
              break;
          }
        }
                
        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(4) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(4) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }
        
    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(5) += var;
  }

  bool PolyTrajOptimizer::swarmGraphGradCostP(const int i_dp,
                                             const double t,
                                             const Eigen::Vector3d &p,
                                             const Eigen::Vector3d &v,
                                             Eigen::Vector3d &gradp,
                                             double &gradt,
                                             double &grad_prev_t,
                                             double &costp)
  {
    // wait all the drones have trajectories
    // if (swarm_trajs_->size() < formation_size_ && drone_id_ != formation_size_-1)
    //   return false;
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;
    
    if (size < formation_size_)
      return false;

    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;
    
    // update the swarm graph
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;
    
    for (size_t id = 0; id < size; id++){
      if (id == drone_id_)
        continue;
        
      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }
    swarm_graph_->updateGraph(swarm_graph_pos);

    // calculate the swarm graph cost and gradp
    double similarity_error;
    swarm_graph_->calcFNorm2(similarity_error);
    

    if (similarity_error > 0){
      ret = true;
    
      // double similarity_error2 = similarity_error * similarity_error;
      // double similarity_error3 = similarity_error2 * similarity_error;
    
      costp = wei_formation_ * similarity_error;
      vector<Eigen::Vector3d> swarm_grad;
      swarm_graph_->getGrad(swarm_grad);
      // double dJ_df = wei_formation_ * 3 * similarity_error2;
      gradp  = wei_formation_ * swarm_grad[drone_id_];

      for (size_t id=0; id<size; id++){
        gradt += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
        if (id != drone_id_)
          grad_prev_t += wei_formation_ * swarm_grad[id].dot(swarm_graph_vel[id]);
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGatherCostGradP(const int i_dp,
                                               const double t,
                                               const Eigen::Vector3d &p,
                                               const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradp,
                                               double &gradt,
                                               double &grad_prev_t,
                                               double &costp)
  {
    // there is a bug still need to fix: when drone_id = formation_size_ - 1, 
    // swarm_trajs_->size() = formation_size_ -1 but not swarm_trajs_->size() = formation_size_
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    
    int size = swarm_trajs_->size();
    if (drone_id_ == formation_size_ - 1)
      size = formation_size_;
    
    if (size < formation_size_)
      return false;

    // for the formation problem, we may optimize the whole trajectory ? 
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    
    // init
    bool ret = false;
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    // get the swarm pos and vel
    Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
    double pt_time = t_now_ + t;
    vector<Eigen::Vector3d> swarm_graph_pos(formation_size_), swarm_graph_vel(formation_size_);
    swarm_graph_pos[drone_id_] = p;
    swarm_graph_vel[drone_id_] = v;

    for (size_t id = 0; id < size; id++){
      if (id == drone_id_)
        continue;
      
      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_graph_pos[id] = swarm_p;
      swarm_graph_vel[id] = swarm_v;
    }

    double alpha1 = double(1.0 - formation_size_)/double(formation_size_);
    double alpha2 = double(1.0/formation_size_);

    for (size_t id = 0; id < size; id++)
      center_pos += swarm_graph_pos[id];
    
    center_pos = center_pos * alpha2;

    // calculate the swarm gathering cost and grad
    const double CLEARANCE2  = swarm_gather_threshold_ * swarm_gather_threshold_ ;
    Eigen::Vector3d dist_vec = center_pos - p;
    double dist2        = dist_vec.norm() * dist_vec.norm();
    double dist_error   = dist2 - CLEARANCE2;
    double dist_error2  = dist_error * dist_error;
    double dist_error3  = dist_error2 * dist_error;
    ret = true;

    if (dist_error3 > 0){
      costp  = wei_gather_ * dist_error3;
      double dJ_df = wei_gather_ * 3 * dist_error2;
      gradp = dJ_df * 2 * alpha1 * dist_vec;
      
      for (size_t id=0; id<size; id++){
        if (id == drone_id_)
          gradt += dJ_df * 2 * alpha1 * dist_vec.dot(swarm_graph_vel[id]);
        else
          gradt += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);

        if (id != drone_id_)
          grad_prev_t += dJ_df * 2 * alpha2 * dist_vec.dot(swarm_graph_vel[id]);
      }
    }
    return ret;
  }
  
  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    // if (i_dp == 0)
    //   return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    // for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    // {
    //   Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
    //   double dist = ray.dot(cps_.direction[i_dp][j]);
    //   double dist_err = obs_clearance_ - dist;
    //   Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

    //   if (dist_err > 0)
    //   {
    //     ret = true;
    //     costp += wei_obs_ * pow(dist_err, 3);
    //     gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
    //   }
    // }
    
    // use esdf
    double dist;
    grid_map_->evaluateEDT(p, dist);
    double dist_err = obs_clearance_ - dist;
    if (dist_err > 0){
      ret = true;
      Eigen::Vector3d dist_grad;
      grid_map_->evaluateFirstGrad(p, dist_grad);

      costp = wei_obs_ * pow(dist_err, 3);
      gradp = - wei_obs_ * 3.0 * pow(dist_err, 2) * dist_grad;
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    // if (i_dp <= 0)
    //   return false;

    bool ret = false;
  
    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

        Eigen::Vector3d swarm_p, swarm_v;
        if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
          swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                    exceed_time * swarm_v;
        }
        Eigen::Vector3d dist_vec = p - swarm_p;
        double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
        double dist2_err = CLEARANCE2 - ellip_dist2;
        double dist2_err2 = dist2_err * dist2_err;
        double dist2_err3 = dist2_err2 * dist2_err;

        if (dist2_err3 > 0)
        {
          ret = true;

          costp += wei_swarm_ * dist2_err3;

          Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
          gradp += dJ_dP;
          gradt += dJ_dP.dot(v - swarm_v);
          grad_prev_t += dJ_dP.dot(-swarm_v);
        }

        if (min_ellip_dist2_ > ellip_dist2)
        {
          min_ellip_dist2_ = ellip_dist2;
        }
      }

    return ret;    
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

    void PolyTrajOptimizer::astarWithMinTraj(const Eigen::MatrixXd &iniState,
                                             const Eigen::MatrixXd &finState, 
                                             vector<Eigen::Vector3d> &simple_path,
                                             Eigen::MatrixXd &ctl_points,
                                             poly_traj::MinJerkOpt &frontendMJ){
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos   = finState.col(0);
    
    /* astar search and get the simple path*/
    simple_path = a_star_->astarSearchAndGetSimplePath(grid_map_->getResolution(), start_pos, end_pos);
    
    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1; 
    Eigen::MatrixXd innerPts;
    
    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for(int i=0; i<piece_num-1; i++)
        innerPts.col(i) = simple_path[i+1];
    } else {
      // piece_num == 1
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2;
    }

    frontendMJ.reset(iniState, finState, piece_num);

    // debug
    // cout <<"----------------" << endl;
    // cout << "iniState : " << iniState.col(0).transpose() << endl;
    // cout << "finState : " << finState.col(0).transpose() << endl;
    // cout << "piece_num : " << piece_num << endl;
    // cout << "simple_path : " << endl;
    // for (int k=0; k<simple_path.size();k++){
    //   cout <<  simple_path[k].transpose() << endl;
    // }
    // cout << "innerPts : " << innerPts << endl;

    /* generate init traj*/
    double des_vel = max_vel_;
    Eigen::VectorXd time_vec(piece_num);
    int debug_num = 0 ;
    do
    {
      for (size_t i = 1; i <= piece_num; ++i)
      {
        time_vec(i-1) = (i == 1) ? (simple_path[1] - start_pos).norm() / des_vel
                               : (simple_path[i] - simple_path[i - 1]).norm() / des_vel;
      }
      frontendMJ.generate(innerPts, time_vec);
      debug_num++;
      des_vel /= 1.5;
    } while (frontendMJ.getTraj().getMaxVelRate() > max_vel_ && debug_num < 1);

    // debug
    // if (frontendMJ.getTraj().getMaxVelRate() > max_vel_){
    //   cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    //   int num = simple_path.size();
    //   cout << "path : " << endl;
    //   for (int i=0; i< num; i++)
    //     cout << simple_path[i].transpose() << "  ";
    //   cout << endl;
    //   cout << "time_vec : " << time_vec.transpose() << endl;
    //   cout << "max vel : " << frontendMJ.getTraj().getMaxVelRate() << endl;
    // }
    
    poly_traj::Trajectory traj;
    traj = frontendMJ.getTraj();

    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
    
    // return traj;
    // return frontendMJ;
  }

  bool PolyTrajOptimizer::getFormationPos(vector<Eigen::Vector3d> &swarm_graph_pos, Eigen::Vector3d pos){
    if (swarm_trajs_->size() < formation_size_ || !use_formation_){
      return false;  
    } 
    else
    {
      double pt_time = t_now_;
      
      swarm_graph_pos[drone_id_] = pos;

      for (size_t id = 0; id < swarm_trajs_->size(); id++){
        if (swarm_trajs_->at(id).drone_id < 0 || swarm_trajs_->at(id).drone_id == drone_id_)
          continue;
        
        double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

        Eigen::Vector3d swarm_p, swarm_v;
        if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
        {
          swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
          swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
        }
        else
        {
          double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
          swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
          swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                    exceed_time * swarm_v;
        }
        swarm_graph_pos[id] = swarm_p;
      }
      return true;
    }
  }

  void PolyTrajOptimizer::showFormationInformation(bool is_show, Eigen::Vector3d pos){
    if(!is_show)
      return;
    
    if (swarm_trajs_->size() < formation_size_ || drone_id_ != 0 || !use_formation_){
      return;  
    } else{
      vector<Eigen::Vector3d> swarm_graph_pos(formation_size_);
      getFormationPos(swarm_graph_pos, pos);
      swarm_graph_->updateGraph(swarm_graph_pos);
      // calculate the swarm graph cost and gradp
      double similarity_error;
      swarm_graph_->calcFNorm2(similarity_error);
      cout << "[formation]: --------------- " << similarity_error << " --------------" << endl;
    }
  }

  bool PolyTrajOptimizer::leaderPosFormationCostGradP(const int i_dp,
                                                      const double t,
                                                      const Eigen::Vector3d &p,
                                                      const Eigen::Vector3d &v,
                                                      Eigen::Vector3d &gradp,
                                                      double &gradt,
                                                      double &grad_prev_t,
                                                      double &costp)
  {
    if (drone_id_ == 0)
      return false;
    
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;

    // init parameters
    gradp.setZero();
    costp = 0;
    gradt = 0;
    grad_prev_t = 0;

    /* formation cost */
    // get pilot_pos and pilot_vel
    Eigen::Vector3d pilot_pos, pilot_vel;
    double pt_time = t_now_ + t;
    double traj_i_satrt_time = swarm_trajs_->at(0).start_time;
    if (pt_time < traj_i_satrt_time + swarm_trajs_->at(0).duration){
      pilot_pos = swarm_trajs_->at(0).traj.getPos(pt_time - traj_i_satrt_time);
      pilot_vel = swarm_trajs_->at(0).traj.getVel(pt_time - traj_i_satrt_time);
    } else {
      double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(0).duration);
      pilot_vel = swarm_trajs_->at(0).traj.getVel(swarm_trajs_->at(0).duration);
      pilot_pos = swarm_trajs_->at(0).traj.getPos(swarm_trajs_->at(0).duration) + exceed_time * pilot_vel;
    }

    // get formation goal
    Eigen::Vector3d formation_goal = pilot_pos + formation_relative_dist_[drone_id_];
    double formation_clearance = 0.1;

    // calculate cost
    Eigen::Vector3d dist_vec = p - formation_goal;
    double dist2 = dist_vec.squaredNorm();    
    
    double dist2_err_max  = dist2 - formation_clearance * formation_clearance;
    double dist2_err2_max = dist2_err_max * dist2_err_max;
    double dist2_err3_max = dist2_err2_max * dist2_err_max;

    if (dist2_err_max > 0 ){
      costp = wei_formation_ * dist2_err3_max;
      Eigen::Vector3d dJ_dP = wei_formation_ * 3 * dist2_err2_max * 2 * dist_vec;
      gradp = dJ_dP;
      gradt = dJ_dP.dot(v - pilot_vel);
      grad_prev_t = dJ_dP.dot(-pilot_vel);
    }
    return true;
  }

  bool PolyTrajOptimizer::relativePosFormationCostGradP(const int i_dp,
                                                        const double t,
                                                        const Eigen::Vector3d &p,
                                                        const Eigen::Vector3d &v,
                                                        Eigen::Vector3d &gradp,
                                                        double &gradt,
                                                        double &grad_prev_t,
                                                        double &costp)
  {
    if (drone_id_ == 0)
      return false;
    
    if (i_dp <= 0 || i_dp >= cps_.cp_size * 2 / 3)
      return false;
    
    int formation_size = 7;
    int size = swarm_trajs_->size();
    if (size < formation_size)
      return false;

    // init parameters
    gradp.setZero();
    costp = 0;
    gradt = 0;
    grad_prev_t = 0;

    // get the swarm pos
    vector<Eigen::Vector3d> swarm_pos(formation_size), swarm_vel(formation_size);
    swarm_pos[drone_id_] = p;
    swarm_vel[drone_id_] = v;
    
    for (size_t id = 0; id < formation_size; id++){
      if (id == drone_id_)
        continue;
      
      double pt_time = t_now_ + t;
      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time - traj_i_satrt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time - traj_i_satrt_time);
      }
      else
      {
        double exceed_time = pt_time - (traj_i_satrt_time + swarm_trajs_->at(id).duration);
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      swarm_pos[id] = swarm_p;
      swarm_vel[id] = swarm_v;
    }

    // get relative goal and connect_num
    int connect_num;
    vector<Eigen::Vector3d> connect_pos, connect_vel;
    if (drone_id_ == 0){
      connect_num = 6;
      for (int i=1; i<= connect_num; i++){
        connect_pos.push_back(swarm_pos[i]-formation_relative_dist_[i]);
        connect_vel.push_back(swarm_vel[i]);
      }
    }     
    else{ 
      connect_num = 3;
      if (drone_id_ == 1){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[6]+formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[6]);
        connect_pos.push_back(swarm_pos[2]+formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[2]);
      } else if (drone_id_ == 2){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[1]+formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[1]);
        connect_pos.push_back(swarm_pos[3]+formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[3]);
      } else if (drone_id_ == 3){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[2]+formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[2]);
        connect_pos.push_back(swarm_pos[4]+formation_relative_dist_[2]);
        connect_vel.push_back(swarm_vel[4]);
      } else if (drone_id_ == 4){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[3]+formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[3]);
        connect_pos.push_back(swarm_pos[5]+formation_relative_dist_[3]);
        connect_vel.push_back(swarm_vel[5]);
      } else if (drone_id_ == 5){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[4]+formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[4]);
        connect_pos.push_back(swarm_pos[6]+formation_relative_dist_[4]);
        connect_vel.push_back(swarm_vel[6]);
      } else if (drone_id_ == 6){
        connect_pos.push_back(swarm_pos[0]+formation_relative_dist_[6]);
        connect_vel.push_back(swarm_vel[0]);
        connect_pos.push_back(swarm_pos[5]+formation_relative_dist_[1]);
        connect_vel.push_back(swarm_vel[5]);
        connect_pos.push_back(swarm_pos[1]+formation_relative_dist_[5]);
        connect_vel.push_back(swarm_vel[1]);
      }
    }
    
    // calculate cost
    double formation_clearance = 0.1;
    for (int i=0; i<connect_num; i++){
      Eigen::Vector3d dist_vec = p - connect_pos[i];
      double dist2 = dist_vec.squaredNorm();    
      
      double dist2_err_max  = dist2 - formation_clearance * formation_clearance;
      double dist2_err2_max = dist2_err_max * dist2_err_max;
      double dist2_err3_max = dist2_err2_max * dist2_err_max;

      if (dist2_err_max > 0 ){
        costp += wei_formation_ * dist2_err3_max;
        Eigen::Vector3d dJ_dP = wei_formation_ * 3 * dist2_err2_max * 2 * dist_vec;
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - connect_vel[i]);
        grad_prev_t += dJ_dP.dot(-connect_vel[i]);
      }
    }
    return true;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constrain_points_perPiece",  cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle",            wei_obs_, -1.0);
    nh.param("optimization/weight_swarm",               wei_swarm_, -1.0);
    nh.param("optimization/weight_feasibility",         wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance",         wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time",                wei_time_, -1.0);
    nh.param("optimization/weight_formation",           wei_formation_, -1.0);
    nh.param("optimization/weight_gather",              wei_gather_, -1.0);

    nh.param("optimization/obstacle_clearance",         obs_clearance_, -1.0);
    nh.param("optimization/swarm_clearance",            swarm_clearance_, -1.0);
    nh.param("optimization/swarm_gather_threshold",     swarm_gather_threshold_, -1.0);
    nh.param("optimization/formation_type",             formation_type_, -1);
    nh.param("optimization/formation_method_type",      formation_method_type_, 0);
    nh.param("optimization/max_vel",                    max_vel_, -1.0);
    nh.param("optimization/max_acc",                    max_acc_, -1.0);
    
    // set the formation type
    swarm_graph_.reset( new SwarmGraph );
    setDesiredFormation(formation_type_);
    
    /* ros utils, API prepare for futrue formation assginment work */
    // re_assignment_sub_ = nh.subscribe("/swarm_formation_assignment", 1, &PolyTrajOptimizer::reAssignmentCallback, this);
    // assignment_pub_    = nh.advertise<traj_utils::Assignment>("/swarm_formation_assignment", 1);
    
    // benchmark position-based formation setting
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(2.6, -1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, -3.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(-2.6, -1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(-2.6, 1.5, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 3.0, 0.0));
    formation_relative_dist_.push_back(Eigen::Vector3d(2.6, 1.5, 0.0));  
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(5.2, -3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, -6.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(-5.2, -3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(-5.2, 3.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(0.0, 6.0, 0.0));
    // formation_relative_dist_.push_back(Eigen::Vector3d(5.2, 3.0, 0.0));    
  }

  void PolyTrajOptimizer::reAssignmentCallback(const traj_utils::AssignmentConstPtr &msg){
    vector<int> assignment(formation_size_);
    printf("\033[47;34mGet reAssignment from other UAV~\n.\033[0m");
    cout << "assignment is : ";
    for (int i=0; i<formation_size_; i++){
      assignment[i] = msg->assignment[i];
      cout << assignment[i] << " ";
    }
    cout << endl;
    
    swarm_graph_->setDesiredAssignment(assignment);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(800, 200, 40));

  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }
  
}