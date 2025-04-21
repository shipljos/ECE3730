#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ros_interfaces/srv/get_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

namespace chr = std::chrono;
using namespace BT;
using Pose = geometry_msgs::msg::Pose;
// using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// Simple function that return a NodeStatus
NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return NodeStatus::SUCCESS;
}

// SyncActionNode (synchronous action) with an input port.
/*
   a SyncActionNode can only return SUCCESS or FAILURE
   it cannot return RUNNING. so it is only good for simple, fast operations
 */
// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        SaySomething(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<std::string>("message") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [message]: ", 
                        msg.error() );
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};
class Wait : public StatefulActionNode
{
    public:
        Wait(const std::string& name, const NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}
        static PortsList providedPorts()
        {
            return{ InputPort<unsigned>("seconds") };
        }
        NodeStatus onStart() override;
        NodeStatus onRunning() override;
        void onHalted() override;
    private:
    	unsigned _seconds;
        chr::system_clock::time_point _completion_time;
};
NodeStatus Wait::onStart()
{
    if ( !getInput<unsigned>("seconds", _seconds))
    {
        throw RuntimeError("missing required input [seconds]");
    }
    printf("[ Wait: ] seconds = %d\n",_seconds);
    _completion_time = chr::system_clock::now() + chr::milliseconds(_seconds*1000);
    return NodeStatus::RUNNING;
}

NodeStatus Wait::onRunning()
{
    std::this_thread::sleep_for(chr::milliseconds(1000));
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ Wait: FINISHED ]" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}
void Wait::onHalted()
{
    printf("[ Wait: ABORTED ]");
}
using Spin = nav2_msgs::action::Spin;
class SpinAction: public RosActionNode<Spin>
{
    public:
        SpinAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<Spin>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({InputPort<float>("target_yaw")});
        }
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("target_yaw", goal.target_yaw);
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            std::cout << "spinning result "  << wr.result->error_msg << "\n";
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            std::cout << "spinning " << error << "\n";
            return NodeStatus::FAILURE;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            std::cout << "spinning feedback " << feedback->angular_distance_traveled << "\n";
            return NodeStatus::RUNNING;
        }
};

using GetPose = ros_interfaces::srv::GetPose;
class PoseService: public RosServiceNode<GetPose>
{
    public:
    PoseService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<GetPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return {OutputPort<Pose>("pose")};
        }
        bool setRequest(Request::SharedPtr& request) override
        {
            std::cout<< "Request from PoseService:" << request <<"\n";
            return true;
        }
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            setOutput("pose",response->pose); //from ros_interfaces/srv/GetPose.srv
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            std::cout<< "Error in GetPose:" << error <<"\n";
            return NodeStatus::FAILURE;
        }
};

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using Path = nav_msgs::msg::Path;
class ComputePathToPoseAction: public RosActionNode<ComputePathToPose>
{
    public:
    ComputePathToPoseAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<ComputePathToPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({
                InputPort<Pose>("goal_pose"),
                InputPort<Pose>("start_pose"),
                OutputPort<uint16_t>("error_code"),
                OutputPort<Path>("path")});
        }
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("goal_pose", goal.goal.pose);
            getInput("start_pose", goal.goal.pose);
            goal.goal.header.frame_id="map";
            goal.start.header.frame_id="map";
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            //RCLCPP_INFO(node_->get_logger(), "ComputePathToPose Goal reached!");
            std::cout << "ComputePathToPose result " << wr.result->error_code << " " << wr.result->error_msg << "\n";
            setOutput("path", wr.result->path);
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
        // for some reason in v4 this will not compile
	    //RCLCPP_ERROR(node_->get_logger(), "Error in Result Recieved: %d", error);
            // for some reason, it seems that nav to pose always returns fail?
            // it times out as a known bug...
            // https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/76
        	std::cout<< "Error in Result Recieved:" <<  error <<"\n";
            // should return fail, but then the tree aborts.
            //return NodeStatus::FAILURE;
            // so return success and figure it out in the behavours
            // with delays?
            return NodeStatus::SUCCESS;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
        	std::cout<< "Compute path to Pose feedback Recieved:" <<  feedback <<"\n";
            return NodeStatus::RUNNING;
        }
};

using NavigateToPose = nav2_msgs::action::NavigateToPose;
class NavigateToPoseAction: public RosActionNode<NavigateToPose>
{
    public:
        NavigateToPoseAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<NavigateToPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({
                InputPort<Pose>("goal_pose")});
        }
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("goal_pose", goal.pose.pose);
            goal.pose.header.frame_id="map";
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            //RCLCPP_INFO(node_->get_logger(), "NavigateToPose Goal reached!");
		std::cout << "NavigateToPose Goal reached! " << wr.result << "\n";
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
        // for some reason in v4 this will not compile
	    //RCLCPP_ERROR(node_->get_logger(), "Error in Result Recieved: %d", error);
            // for some reason, it seems that nav to pose always returns fail?
            // it times out as a known bug...
            // https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/76
        	std::cout<< "Error in Result Recieved:" <<  error <<"\n";
            // should return fail, but then the tree aborts.
            //return NodeStatus::FAILURE;
            // so return success and figure it out in the behavours
            // with delays?
            return NodeStatus::SUCCESS;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
        	std::cout<< "Nav to pose feedback Recieved:" <<  feedback <<"\n";
            return NodeStatus::RUNNING;
        }
};
using GetMap = nav_msgs::srv::GetMap;
class MapService: public RosServiceNode<GetMap>
{
    public:
        MapService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<GetMap>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({OutputPort<OccupancyGrid>("map")});
        }
        bool setRequest(Request::SharedPtr& request) override
        {
            std::cout<< "Request from MapService:" << request <<"\n";
            return true;
        }
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            setOutput("map",response->map); //from nav_msgs/srv/GetMap.srv
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            std::cout<< "Error in NodeStatus:" << error <<"\n";
            return NodeStatus::FAILURE;
        }
};

class FindFrontierNode : public SyncActionNode
{
public:
  FindFrontierNode(const std::string& name, const NodeConfig& config)
  : SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("find_frontier_node");
  }

  static PortsList providedPorts()
  {
    return {
      OutputPort<Pose>("goal_pose"),
      InputPort<Pose>("current_pose"),
      InputPort<OccupancyGrid>("map")
    };
  }

  NodeStatus tick() override
  {
    auto pose_res = getInput<Pose>("current_pose");
    auto map_res = getInput<OccupancyGrid>("map");

    if (!pose_res || !map_res)
    {
      RCLCPP_ERROR(node_->get_logger(), "Missing required inputs");
      return NodeStatus::FAILURE;
    }

    auto frontier = findFurthestReachableFrontier(pose_res.value(), map_res.value());
    if (frontier)
    {
      setOutput("goal_pose", frontier.value());
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;

  bool isFrontier(int x, int y, const OccupancyGrid& map)
  {
    int width = map.info.width;
    int height = map.info.height;
    auto idx = [=](int x, int y) { return y * width + x; };

    if (map.data[idx(x, y)] != 0) return false;

    //Checking values in a 3x3 grid around every point
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        if (dx == 0 && dy == 0) continue;
        int nx = x + dx;
        int ny = y + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
        {
          if (map.data[idx(nx, ny)] == -1)
            return true;
        }
      }
    }
    return false;
  }

  Pose gridToWorld(int x, int y, const OccupancyGrid& map)
  {
    auto resolution = map.info.resolution;
    auto origin = map.info.origin.position;

    Pose pose;
    pose.position.x = origin.x + (x + 0.5) * resolution;
    pose.position.y = origin.y + (y + 0.5) * resolution;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
  }

  double distance(const Point& a, const Point& b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  bool withinYawThreshold(const Pose& p_current, const Pose& p_goal, double angle_tolerance_deg = 30.0)
  {
    tf2::Quaternion q;
    tf2::fromMsg(p_current.orientation, q);
    double current_yaw = tf2::getYaw(q);

    double dx = p_goal.position.x - p_current.position.x;
    double dy = p_goal.position.y - p_current.position.y;
    double goal_direction = std::atan2(dy, dx);

    double angle_diff = goal_direction - current_yaw;

    // Normalize angle to [-π, π]
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

    // Convert tolerance to radians
    double tolerance_rad = angle_tolerance_deg * M_PI / 180.0;

    return std::abs(angle_diff) <= tolerance_rad;
  }

  bool awayFromWall(const Pose& p, const OccupancyGrid& map, double distance_from_wall_threshold = 1.0)
  {
    const auto& info = map.info;

    // Convert pose to map indices
    int fx = static_cast<int>((p.position.x - info.origin.position.x) / info.resolution);
    int fy = static_cast<int>((p.position.y - info.origin.position.y) / info.resolution);

    int radius_cells = static_cast<int>(distance_from_wall_threshold / info.resolution);

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            int x = fx + dx;
            int y = fy + dy;

            // Bounds check
            if (x < 0 || y < 0 || x >= static_cast<int>(info.width) || y >= static_cast<int>(info.height)) {
                continue;
            }

            // Circular distance check
            double distance = std::sqrt(dx * dx + dy * dy) * info.resolution;
            if (distance > distance_from_wall_threshold) continue;

            int index = y * info.width + x;
            if (map.data[index] >= 50) {
                return false;  // Too close to a wall
            }
        }
    }

    return true;  // Frontier is safe
  }

  std::optional<Pose> findFurthestReachableFrontier(
    const Pose& robot_pose,
    const OccupancyGrid& map)
  {
    struct Frontier
    {
      double distance;
      Pose pose;
    };

    std::vector<Frontier> frontiers;
    int width = map.info.width;
    int height = map.info.height;

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        if (isFrontier(x, y, map))
        {
          auto pose = gridToWorld(x, y, map);
          double dist = distance(robot_pose.position, pose.position);
          if (dist>1.6)
          {
            // std::cout<< "Valid frontier is:" << dist <<" meters away.\n";
            frontiers.push_back({dist, pose});
          }
        }
      }
    }

    std::sort(frontiers.begin(), frontiers.end(), [](const Frontier& a, const Frontier& b) {
      return a.distance < b.distance;
    });

    for (const auto& f : frontiers)
    {
        if ((withinYawThreshold(robot_pose,f.pose, 30.0)) && (awayFromWall(f.pose, map, 0.8))){
            std::cout << "I think this is a good pose to go to: x = " 
                      << f.pose.position.x << ", y = "
                      << f.pose.position.y <<"\n";
            return f.pose;
        }
    }
    std::cout<< "Couldn't find a pose to go to..." <<"\n";
    return std::nullopt;
  }
};

class MapFinished : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        MapFinished(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            return { InputPort<OccupancyGrid>("map"),
                    InputPort<Pose>("current_pose"),
                    InputPort<Pose>("home") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<OccupancyGrid>msg = getInput<OccupancyGrid>("map");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [map]: ",msg.error());
            }
            Expected<Pose>home_msg = getInput<Pose>("home");
            // Check if expected is valid. If not, throw its error
            if (!home_msg)
            {
                throw RuntimeError("missing required input [home]: ",home_msg.error());
            }
            Expected<Pose>current_msg = getInput<Pose>("current_pose");
            // Check if expected is valid. If not, throw its error
            if (!current_msg)
            {
                throw RuntimeError("missing required input [current_pose]: ",current_msg.error());
            }
            OccupancyGrid map=msg.value();
            unsigned n=map.info.width*map.info.height;
            unsigned i;
            unsigned cnt=0;
            std::cout<<"Map info: "<< map.info.origin.position.x << ", " 
                << map.info.origin.position.y << ","<<n<<std::endl;
            for(i=0;i<n;++i) {
                if(map.data[i] == -1) {
                    cnt++;
                }
            }
            float percent_done = (float)cnt/(float)n;
            // std::cout << "Map is "<< percent_done << " known"<<std::endl;

            Pose current_pose = current_msg.value();
            Pose home = home_msg.value();
            auto closeness = std::hypot(current_pose.position.x - home.position.x, current_pose.position.y - home.position.y);
            std::cout << "Euclidean Distance from Home Position: "<< closeness <<std::endl;
            if(closeness < 1.5) {
                std::cout << "Map is finished!\n";
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }
};

class SerializeMapAction : public SyncActionNode
{
public:
    SerializeMapAction(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), node_(rclcpp::Node::make_shared("serialize_map_node"))
    {
        client_ = node_->create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    }

    static PortsList providedPorts()
    {
        return{ InputPort<std::string>("map_filename") };
    }

    NodeStatus tick() override
    {
        std::string filename;
        if (!getInput("map_filename", filename))
        {
            RCLCPP_ERROR(node_->get_logger(), "Missing required input: map_filename");
            return NodeStatus::FAILURE;
        }

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for serialize_map service...");
        }

        // Prepare the request
        auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
        request->filename = filename;

        // Call the service asynchronously
        auto future = client_->async_send_request(request,
            std::bind(&SerializeMapAction::handle_response, this, std::placeholders::_1));

        // Wait for the result (You can modify this to handle callbacks more elegantly)
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            if (future.get()) {
                RCLCPP_INFO(node_->get_logger(), "Successfully serialized the map to %s", filename.c_str());
                return NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to serialize the map.");
                return NodeStatus::FAILURE;
            }
        }
        return NodeStatus::FAILURE;
    }

private:
    void handle_response(std::shared_future<std::shared_ptr<slam_toolbox::srv::SerializePoseGraph::Response>> future)
    {
        // Handle any response if needed
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr client_;
};


static const char* mapper = R"(
    <root BTCPP_format="4" >
        <BehaviorTree ID="MainTree">
            <Sequence>
                <Wait seconds="10"/>
                <SaySomething   message="mission started..." />
                <PoseService name="get_pose" pose = "{home_pose}" />
                <SaySomething   message="navigating ..." />
                <RetryUntilSuccessful num_attempts="10">
                    <Sequence>
                        <MapService map = "{map}" />
                        <FindFrontier goal_pose = "{target_pose}" 
                                    current_pose="{home_pose}"
                                    map ="{map}"/>                      
                        <NavigateToPose goal_pose = "{target_pose}"/>
                        <Wait seconds="5"/>
                    </Sequence>
                </RetryUntilSuccessful>
                <RetryUntilSuccessful num_attempts="120">
                    <Sequence>
                        <MapService map = "{map}" />
                        <PoseService name="get_pose" pose = "{current_pose}" />
                        <FindFrontier goal_pose = "{target_pose}" 
                            current_pose="{current_pose}"
                            map ="{map}"/>                        
                        <NavigateToPose goal_pose = "{target_pose}"/>
                        <Wait seconds="1"/>
                        <PoseService name="get_pose" pose = "{current_pose}" />
                        <MapService map = "{map}" /> 
                        <MapFinished map = "{map}" current_pose = "{current_pose}" home = "{home_pose}"/>          
                    </Sequence>
                </RetryUntilSuccessful>
                <NavigateToPose goal_pose = "{home_pose}"/>
                <Wait seconds="10"/>
                <SaySomething   message="mission completed! Saving map..." />
                <SerializeMapAction map_filename="/home/jshipley/Documents/ros2_ws/src/ece3730/robot/config/maps/my_serialized_map" />
            </Sequence>
        </BehaviorTree>
    </root>
    )";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    // these are the simple BT nodes
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<Wait>("Wait");
    factory.registerNodeType<SaySomething>("SaySomething");

    // this is the ROS2 action client node so it needs some extra parameters
    auto spin_node = std::make_shared<rclcpp::Node>("spin_client");
    // provide the ROS node and the name of the action service
    RosNodeParams params; 
    params.nh = spin_node;
    params.default_port_value = "spin";
    factory.registerNodeType<SpinAction>("Spin", params);

    // this is the ROS2 action client node so it needs some extra parameters
    auto nav_node = std::make_shared<rclcpp::Node>("navigate_client");
    // provide the ROS node and the name of the action service
    RosNodeParams nav_params; 
    nav_params.nh = nav_node;
    nav_params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<NavigateToPoseAction>("NavigateToPose", nav_params);

    auto compute_path_node = std::make_shared<rclcpp::Node>("compute_path_client");
    // provide the ROS node and the name of the action service
    RosNodeParams compute_path_params; 
    compute_path_params.nh = compute_path_node;
    compute_path_params.default_port_value = "compute_path_to_pose";
    factory.registerNodeType<ComputePathToPoseAction>("ComputePathToPose",compute_path_params);

    auto map_service_node = std::make_shared<rclcpp::Node>("get_map_client");
    // provide the ROS node and the name of the  service
    RosNodeParams map_service_params; 
    map_service_params.nh = map_service_node;
    map_service_params.default_port_value = "slam_toolbox/dynamic_map";
    factory.registerNodeType<MapService>("MapService", map_service_params);
    
    auto pose_service_node = std::make_shared<rclcpp::Node>("pose_node");
    RosNodeParams pose_service_params; 
    pose_service_params.nh = pose_service_node;
    pose_service_params.default_port_value = "get_pose";
    factory.registerNodeType<PoseService>("PoseService", pose_service_params); 

    factory.registerNodeType<FindFrontierNode>("FindFrontier");
    factory.registerNodeType<MapFinished>("MapFinished");
    factory.registerNodeType<SerializeMapAction>("SerializeMapAction");
    
    // Create the tree using the blackboard
    auto tree = factory.createTreeFromText(mapper);
    // auto tree = factory.createTreeFromText(racer);

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
//    std::cout << "--- ticking\n";
    NodeStatus status = tree.tickOnce();
//    std::cout << "--- status: " << toStr(status) << "\n\n";

    while(status == NodeStatus::RUNNING) 
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(chr::milliseconds(100));

 //       std::cout << "--- ticking\n";
        status = tree.tickOnce();
//        std::cout << "--- status: " << toStr(status) << "\n\n";
    }
    return 0;
}


