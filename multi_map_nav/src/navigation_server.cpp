
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <sqlite3.h>
#include <std_srvs/Trigger.h>
#include <ros/package.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/LoadMap.h>
#include <multi_map_nav/NavigationAction.h>  

typedef actionlib::SimpleActionServer<multi_map_nav::NavigationAction> NavigationServer;
class NavigationActionServer
{
public:
  NavigationActionServer(const ros::NodeHandle& nh) :
    nh_(nh),
    as_(nh_, "navigation_action", boost::bind(&NavigationActionServer::executeCB, this, _1), false)
  {

    std::string db_path = "/home/anmol/wormhole_ws/src/multi_map_nav/sql/wormholes.db" ;
    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
      ROS_ERROR("Failed to open wormholes.db at '%s': %s", db_path.c_str(), sqlite3_errmsg(db_));
      sqlite3_close(db_);
      db_ = nullptr;
    }

    move_base_ac_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
    ROS_INFO("Waiting for move_base action server...");
    move_base_ac_->waitForServer();

    load_map_client_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
    load_map_client_.waitForExistence();

    current_map_ = "room_1";
    nav_msgs::LoadMap init_srv;
    std::string s= "/home/anmol/wormhole_ws/src/multi_map_nav/maps/room_1.yaml";
    init_srv.request.map_url = s;
    if (!load_map_client_.call(init_srv) || init_srv.response.result) {
      ROS_WARN("Could not load initial map 'room_1'—make sure map_server is running with '/load_map'.");
    }
    ros::Duration(1.0).sleep();

    as_.start();
    ROS_INFO("NavigateActionServer ready.");
  }

  ~NavigationActionServer()
  {
    if (db_) {
      sqlite3_close(db_);
    }
  }

private:
  ros::NodeHandle nh_;
  NavigationServer as_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_;
  ros::ServiceClient load_map_client_;
  sqlite3* db_;
  std::string current_map_;

  void executeCB(const multi_map_nav::NavigationGoalConstPtr& goal)
  {
    multi_map_nav::NavigationFeedback feedback;
    multi_map_nav::NavigationResult result;

    const std::string target_map = goal->target_map_name;
    const geometry_msgs::PoseStamped target_pose = goal->target_pose;

    ROS_INFO("Received NavigationGoal: target_map='%s'.", target_map.c_str());

    if (target_map == current_map_) {
      feedback.state = "Already on map '" + target_map + "'. Sending goal to move_base.";
      as_.publishFeedback(feedback);

      bool succeeded = sendMoveBaseGoal(target_pose);
      result.success = succeeded;
      as_.setSucceeded(result);
      return;
    }

    double from_x, from_y, from_z,from_w;
    double to_x, to_y, to_z,to_w;
    if (!queryWormhole(current_map_, target_map, from_x, from_y, from_z, from_w,to_x,to_y,to_z,to_w)) {
      feedback.state = "No wormhole entry found from '" + current_map_ + "' to '" + target_map + "'.";
      ROS_INFO("not found");
      result.success = false;
      as_.setAborted(result);
      return;
    }

    {
      feedback.state = "Navigating to wormhole entry on '" + current_map_ + "'.";
      ROS_INFO("Navigating to wormhole entry on");
      as_.publishFeedback(feedback);

      geometry_msgs::PoseStamped wh_entry;
      wh_entry.header.frame_id = "map";
      wh_entry.header.stamp = ros::Time::now();
      wh_entry.pose.position.x = from_x;
      wh_entry.pose.position.y = from_y;
      // tf::Quaternion q1;
      // q1.setRPY(0, 0, from_yaw);
      wh_entry.pose.orientation.x = 0.0;
      wh_entry.pose.orientation.y = 0.0;
      wh_entry.pose.orientation.z = to_z;
      wh_entry.pose.orientation.w = to_w;

      if (!sendMoveBaseGoal(wh_entry)) {
        feedback.state = "Failed to reach wormhole entry.";
        result.success = false;
        as_.setAborted(result);
        return;
      }
    }

    {
      feedback.state = "Calling /change_map to switch to '" + target_map + "'.";
      as_.publishFeedback(feedback);
    
      nav_msgs::LoadMap srv;
    srv.request.map_url ="/home/anmol/wormhole_ws/src/multi_map_nav/maps/" + target_map +  ".yaml";
      if (!load_map_client_.call(srv) || srv.response.result) {
        feedback.state = "Failed to load map '" + target_map + ".";
        result.success= false;
        as_.setAborted(result);
        return;
      }
      ROS_INFO("Successfully changed map '%s'.", target_map.c_str());

      ros::Duration(1.0).sleep();
      current_map_ = target_map;
    }


    publishInitialPose(to_x,to_y,to_z,to_w);

    ros::Duration(2.0).sleep();

    {
      feedback.state = "Publishing final goal on '" + target_map + "'.";
      as_.publishFeedback(feedback);

      if (!sendMoveBaseGoal(target_pose)) {
        feedback.state = "Failed to reach final goal on '" + target_map + "'.";
        result.success = false;
        as_.setAborted(result);
        return;
      }
    }

    // If we reach here, everything succeeded
    result.success = true;
    as_.setSucceeded(result);
  }


  geometry_msgs::PoseWithCovarianceStamped waitForAMCLPose()
{
  geometry_msgs::PoseWithCovarianceStamped latest_pose;
  boost::mutex mutex;
  boost::condition_variable cond;
  bool received = false;

  ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "/amcl_pose", 1,
    [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
      boost::unique_lock<boost::mutex> lock(mutex);
      latest_pose = *msg;
      received = true;
      cond.notify_one();
    });

  ros::Time start = ros::Time::now();
  boost::unique_lock<boost::mutex> lock(mutex);
  while (!received && ros::ok() && (ros::Time::now() - start < ros::Duration(3.0))) {
    cond.wait_for(lock, boost::chrono::milliseconds(100));
    ros::spinOnce();  // ensure callbacks are processed
  }

  sub.shutdown();
  return latest_pose;
}


  bool sendMoveBaseGoal(const geometry_msgs::PoseStamped& pose)
  {
    if (!move_base_ac_->isServerConnected()) {
      ROS_ERROR("move_base action server not connected.");
      return false;
    }
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = pose;

    move_base_ac_->sendGoal(mb_goal);
    bool finished = move_base_ac_->waitForResult(ros::Duration(60.0));  // wait up to 60s
    if (!finished) {
      move_base_ac_->cancelGoal();
      ROS_WARN("Timed out while waiting for move_base result.");
      return false;
    }
    return (move_base_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  }

  bool queryWormhole(const std::string& from_room,
                     const std::string& to_room,
                     double& from_x, double& from_y, double& from_z, double& from_w,
                     double& to_x,   double& to_y,   double& to_z, double& to_w)
  {
    if (!db_) return false;

    const char* sql = 
      "SELECT from_x, from_y, from_z,from_w, to_x, to_y, to_z, to_w "
      "FROM wormholes_new WHERE  from_room = ? AND to_room = ? LIMIT 1;";

    ROS_INFO("Inside1");

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
      ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
      return false;
    }

    sqlite3_bind_text(stmt, 1, from_room.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, to_room.c_str(),   -1, SQLITE_STATIC);

    bool found = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      from_x = sqlite3_column_double(stmt, 0);
      from_y = sqlite3_column_double(stmt, 1);
      from_z = sqlite3_column_double(stmt, 2);
      from_w = sqlite3_column_double(stmt, 3);
      to_x   = sqlite3_column_double(stmt, 4);
      to_y   = sqlite3_column_double(stmt, 5);
      to_z   = sqlite3_column_double(stmt, 6);
      to_w   = sqlite3_column_double(stmt, 7);

      ROS_INFO("Query successful: from (%f, %f, %f, %f) -> to (%f, %f, %f, %f)", 
          from_x, from_y, from_z, from_w,
          to_x, to_y, to_z, to_w);


      found = true;
    }
    sqlite3_finalize(stmt);
    ROS_INFO("Inside2");

    return found;
  }

  void publishInitialPose(double x, double y, double z, double w)
  {
    ros::Publisher init_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped init;
    init.header.stamp = ros::Time::now();
    init.header.frame_id = "map";
    init.pose.pose.position.x = x;
    init.pose.pose.position.y = y;
    // tf::Quaternion q;
    // q.setRPY(0, 0, yaw);
    ROS_INFO("Waiting for subscribers to /initialpose...");
    while (init_pub.getNumSubscribers() == 0 && ros::ok()) {
        ros::Duration(0.1).sleep();
    }

    init.pose.pose.orientation.x = 0.0;
    init.pose.pose.orientation.y = 0.0;
    init.pose.pose.orientation.z = z;
    init.pose.pose.orientation.w = w;

    init.pose.covariance[0] = 0.25;     // x
    init.pose.covariance[7] = 0.25;     // y
    init.pose.covariance[35] = 0.0685;  // yaw

    init_pub.publish(init);
    ROS_INFO("Published initialpose: [%.2f, %.2f, yaw=%.3f]", x, y, z);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_action_server");
  ros::NodeHandle nh;

  NavigationActionServer server(nh);
  ros::spin();
  return 0;
}



// #include <ros/ros.h>
// #include <actionlib/server/simple_action_server.h>
// #include <bits/stdc++.h>
// #include <sqlite3.h>
// #include <std_srvs/Trigger.h>
// #include <ros/package.h> 
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <tf/transform_datatypes.h>
// #include <nav_msgs/LoadMap.h>
// #include <multi_map_nav/NavigationAction.h>  

// typedef actionlib::SimpleActionServer<multi_map_nav::NavigationAction> NavigationServer;
// class NavigationActionServer
// {
// public:
//   NavigationActionServer(const ros::NodeHandle& nh) :
//     nh_(nh),
//     as_(nh_, "navigation_action", boost::bind(&NavigationActionServer::executeCB, this, _1), false)
//   {

//     std::string db_path = "/home/anmol/wormhole_ws/src/multi_map_nav/sql/wormholes.db" ;
//     if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
//       ROS_ERROR("Failed to open wormholes.db at '%s': %s", db_path.c_str(), sqlite3_errmsg(db_));
//       sqlite3_close(db_);
//       db_ = nullptr;
//     }

//     move_base_ac_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
//     ROS_INFO("Waiting for move_base action server...");
//     move_base_ac_->waitForServer();

//     load_map_client_ = nh_.serviceClient<nav_msgs::LoadMap>("/change_map");
//     load_map_client_.waitForExistence();

//     current_map_ = "room_1";
//     nav_msgs::LoadMap init_srv;
//     std::string s= "/home/anmol/wormhole_ws/src/multi_map_nav/maps/room_1.yaml";
//     init_srv.request.map_url = s;
//     if (!load_map_client_.call(init_srv) || init_srv.response.result) {
//       ROS_WARN("Could not load initial map 'room_1'—make sure map_server is running with '/load_map'.");
//     }
//     ros::Duration(1.0).sleep();

//     as_.start();
//     ROS_INFO("NavigateActionServer ready.");
//   }

//   ~NavigationActionServer()
//   {
//     if (db_) {
//       sqlite3_close(db_);
//     }
//   }

// private:
//   ros::NodeHandle nh_;
//   NavigationServer as_;
//   std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_;
//   ros::ServiceClient load_map_client_;
//   sqlite3* db_;
//   std::string current_map_;

//   void executeCB(const multi_map_nav::NavigationGoalConstPtr& goal)
//   {
//     multi_map_nav::NavigationFeedback feedback;
//     multi_map_nav::NavigationResult result;

//     const std::string target_map = goal->target_map_name;
//     const geometry_msgs::PoseStamped target_pose = goal->target_pose;

//     ROS_INFO("Received NavigationGoal: target_map='%s'.", target_map.c_str());

//     if (target_map == current_map_) {
//       feedback.state = "Already on map '" + target_map + "'. Sending goal to move_base.";
//       as_.publishFeedback(feedback);

//       bool succeeded = sendMoveBaseGoal(target_pose);
//       result.success = succeeded;
//       as_.setSucceeded(result);
//       return;
//     }

//     double from_x, from_y, from_yaw;
//     double to_x, to_y, to_yaw;
//     // if (!queryWormhole(current_map_, target_map, from_x, from_y, from_yaw, to_x, to_y, to_yaw)) {
//     //   feedback.state = "No wormhole entry found from '" + current_map_ + "' to '" + target_map + "'.";
//     //   result.success = false;
//     //   as_.setAborted(result);
//     //   return;
//     // }
//     std::vector<std::string> path;
//     if (!findPathBetweenMaps(current_map_, target_map, path)) {
//       feedback.state = "No path found from '" + current_map_ + "' to '" + target_map + "'.";
//       result.success = false;
//       as_.setAborted(result);
//       return;
//     }

//     for (size_t i = 1; i < path.size(); ++i) {
//         std::string from = path[i - 1];
//         std::string to   = path[i];
      
//         double from_x, from_y, from_yaw;
//         double to_x, to_y, to_yaw;
      
//         if (!queryWormhole(from, to, from_x, from_y, from_yaw, to_x, to_y, to_yaw)) {
//           feedback.state = "Missing wormhole from '" + from + "' to '" + to + "'";
//           result.success = false;
//           as_.setAborted(result);
//           return;
//         }
      
//         feedback.state = "Navigating to wormhole from '" + from + "' to '" + to + "'";
//         as_.publishFeedback(feedback);
      
//         geometry_msgs::PoseStamped entry_pose;
//         entry_pose.header.frame_id = "map";
//         entry_pose.header.stamp = ros::Time::now();
//         entry_pose.pose.position.x = from_x;
//         entry_pose.pose.position.y = from_y;
//         tf::Quaternion q;
//         q.setRPY(0, 0, from_yaw);
//         entry_pose.pose.orientation.x = q.x();
//         entry_pose.pose.orientation.y = q.y();
//         entry_pose.pose.orientation.z = q.z();
//         entry_pose.pose.orientation.w = q.w();
      
//         if (!sendMoveBaseGoal(entry_pose)) {
//           feedback.state = "Failed to reach wormhole at '" + from + "'";
//           result.success = false;
//           as_.setAborted(result);
//           return;
//         }
      
//         // Change map
//         nav_msgs::LoadMap srv;
//         srv.request.map_url = "/home/anmol/wormhole_ws/src/multi_map_nav/maps/" + to + ".yaml";
//         if (!load_map_client_.call(srv) || srv.response.result) {
//           feedback.state = "Failed to load map '" + to + "'";
//           result.success = false;
//           as_.setAborted(result);
//           return;
//         }
      
//         current_map_ = to;
//         ros::Duration(1.0).sleep();  // small delay after map change
//       }
      

//     {
//       feedback.state = "Navigating to wormhole entry on '" + current_map_ + "'.";
//       ROS_INFO("Navigating to wormhole entry on");
//       as_.publishFeedback(feedback);

//       geometry_msgs::PoseStamped wh_entry;
//       wh_entry.header.frame_id = "map";
//       wh_entry.header.stamp = ros::Time::now();
//       wh_entry.pose.position.x = from_x;
//       wh_entry.pose.position.y = from_y;
//       tf::Quaternion q1;
//       q1.setRPY(0, 0, from_yaw);
//       wh_entry.pose.orientation.x = q1.x();
//       wh_entry.pose.orientation.y = q1.y();
//       wh_entry.pose.orientation.z = q1.z();
//       wh_entry.pose.orientation.w = q1.w();

//       if (!sendMoveBaseGoal(wh_entry)) {
//         feedback.state = "Failed to reach wormhole entry.";
//         result.success = false;
//         as_.setAborted(result);
//         return;
//       }
//     }

//     {
//       feedback.state = "Calling /change_map to switch to '" + target_map + "'.";
//       as_.publishFeedback(feedback);
    
//       nav_msgs::LoadMap srv;
//     srv.request.map_url ="/home/anmol/wormhole_ws/src/multi_map_nav/maps/" + target_map +  ".yaml";
//       if (!load_map_client_.call(srv) || srv.response.result) {
//         feedback.state = "Failed to load map '" + target_map + ".";
//         result.success= false;
//         as_.setAborted(result);
//         return;
//       }
//       ROS_INFO("Successfully changed map '%s'.", target_map.c_str());

//       ros::Duration(1.0).sleep();
//       current_map_ = target_map;
//     }

//     // {
//     //   publishInitialPose(to_x, to_y, to_yaw);
//     //   ros::Duration(0.5).sleep();  // give AMCL a bit more time
//     // }


//     {
//       feedback.state = "Publishing final goal on '" + target_map + "'.";
//       as_.publishFeedback(feedback);

//       if (!sendMoveBaseGoal(target_pose)) {
//         feedback.state = "Failed to reach final goal on '" + target_map + "'.";
//         result.success = false;
//         as_.setAborted(result);
//         return;
//       }
//     }

//     // If we reach here, everything succeeded
//     result.success = true;
//     as_.setSucceeded(result);
//   }

//   bool sendMoveBaseGoal(const geometry_msgs::PoseStamped& pose)
//   {
//     if (!move_base_ac_->isServerConnected()) {
//       ROS_ERROR("move_base action server not connected.");
//       return false;
//     }
//     move_base_msgs::MoveBaseGoal mb_goal;
//     mb_goal.target_pose = pose;

//     move_base_ac_->sendGoal(mb_goal);
//     bool finished = move_base_ac_->waitForResult(ros::Duration(60.0));  // wait up to 60s
//     if (!finished) {
//       move_base_ac_->cancelGoal();
//       ROS_WARN("Timed out while waiting for move_base result.");
//       return false;
//     }
//     return (move_base_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
//   }

//   bool queryWormhole(const std::string& from_map,
//                      const std::string& to_map,
//                      double& from_x, double& from_y, double& from_yaw,
//                      double& to_x,   double& to_y,   double& to_yaw)
//   {
//     if (!db_) return false;

//     const char* sql = 
//       "SELECT from_x, from_y, from_yaw, to_x, to_y, to_yaw "
//       "FROM wormholes WHERE from_map = ? AND to_map = ? LIMIT 1;";

//     sqlite3_stmt* stmt = nullptr;
//     if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
//       ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
//       return false;
//     }

//     sqlite3_bind_text(stmt, 1, from_map.c_str(), -1, SQLITE_STATIC);
//     sqlite3_bind_text(stmt, 2, to_map.c_str(),   -1, SQLITE_STATIC);

//     bool found = false;
//     if (sqlite3_step(stmt) == SQLITE_ROW) {
//       from_x   = sqlite3_column_double(stmt, 0);
//       from_y   = sqlite3_column_double(stmt, 1);
//       from_yaw = sqlite3_column_double(stmt, 2);
//       to_x     = sqlite3_column_double(stmt, 3);
//       to_y     = sqlite3_column_double(stmt, 4);
//       to_yaw   = sqlite3_column_double(stmt, 5);
//       found = true;
//     }
//     sqlite3_finalize(stmt);
//     return found;
//   }


//   bool findPathBetweenMaps(const std::string& start, const std::string& goal, std::vector<std::string>& map_path) {
//     if (!db_) return false;
  
//     std::map<std::string, std::vector<std::string>> graph;
//     const char* sql = "SELECT from_map, to_map FROM wormholes";
//     sqlite3_stmt* stmt = nullptr;
  
//     if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
//       ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db_));
//       return false;
//     }
  
//     // Build the graph
//     while (sqlite3_step(stmt) == SQLITE_ROW) {
//       std::string from = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
//       std::string to   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
//       graph[from].push_back(to);
//       graph[to].push_back(from); // bidirectional
//     }
//     sqlite3_finalize(stmt);
  
//     // BFS
//     std::queue<std::vector<std::string>> q;
//     std::set<std::string> visited;
  
//     q.push({start});
//     visited.insert(start);
  
//     while (!q.empty()) {
//       auto path = q.front(); q.pop();
//       const std::string& last = path.back();
  
//       if (last == goal) {
//         map_path = path;
//         return true;
//       }
  
//       for (const auto& neighbor : graph[last]) {
//         if (visited.find(neighbor) == visited.end()) {
//           visited.insert(neighbor);
//           auto new_path = path;
//           new_path.push_back(neighbor);
//           q.push(new_path);
//         }
//       }
//     }
//     return false;
//   }
  

//   void publishInitialPose(double x, double y, double yaw)
//   {
//     ros::Publisher init_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
//       "/initialpose", 1, true);
//     geometry_msgs::PoseWithCovarianceStamped init;
//     init.header.stamp = ros::Time::now();
//     init.header.frame_id = "map";
//     init.pose.pose.position.x = x;
//     init.pose.pose.position.y = y;
//     tf::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     init.pose.pose.orientation.x = q.x();
//     init.pose.pose.orientation.y = q.y();
//     init.pose.pose.orientation.z = q.z();
//     init.pose.pose.orientation.w = q.w();
//     init_pub.publish(init);
//     ROS_INFO("Published initialpose: [%.2f, %.2f, yaw=%.3f]", x, y, yaw);
//   }
// };


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "navigation_action_server");
//   ros::NodeHandle nh;

//   NavigationActionServer server(nh);
//   ros::spin();
//   return 0;
// }

