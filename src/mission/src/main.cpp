#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/rotate.hpp"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/move_to.hpp"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/detect_object.hpp"


using namespace std;
using namespace std::chrono_literals;


// This node gets the information from detector.py and transfers it over to the rest 
// (acts as a vessel that extracts information, we need to pass it onto the next node as it keeps getting updated)

// Variable definitions
string detections;

class DetectionNodeConfig : public rclcpp::Node
{
public:
    DetectionNodeConfig(BT::Blackboard::Ptr blackboard)
    : Node("mission_node"), blackboard_(blackboard)

    {
        sub_ = this->create_subscription<std_msgs::msg::String>("vision_subscriber", 10, 
            std::bind(&DetectionNodeConfig::callback, this, std::placeholders::_1));  // recieves the message     
    }

    void callback(const std_msgs::msg::String::SharedPtr sub_msg)
    {
        blackboard_ -> set("detections", sub_msg->data);  // Set the detections in the blackboard
        cout << sub_msg->data << endl;  // Print the detections received from the detector.py
    }


private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    BT::Blackboard::Ptr blackboard_;
};


// Submerge (Action Node)
class Submerge : public BT::SyncActionNode 
{
public: 
    explicit Submerge(const std::string &name, const BT::NodeConfig &config)
     : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<string>("detections") };
    }

    BT::NodeStatus tick() override {
        auto res = getInput<std::string>("detections");
        if (res = "0")
        {
            cout << "Sawfish detected" << endl; 
            return BT::NodeStatus::SUCCESS;
        }
        if (res = "1")
        {
            cout << "Reefshark detected" << endl; 
            return BT::NodeStatus::SUCCESS;
        }
        if (res = "No detections")
        {
            cout << "You're a bum" << endl; 
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
        
    }
};
 
// Center_sub_perpendicular_to_gate (Action Node)
class Center_sub_perpendicular_to_gate : public BT::SyncActionNode
{
public: 
    explicit Center_sub_perpendicular_to_gate(const std::string &name) : BT::SyncActionNode(name, {})
    {
 
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Center_sub_perpendicular_to_gate" << endl;
        return BT::NodeStatus::SUCCESS; 
    }
};
 
// Turn_right_90_deg (Action Node)
class TurnRight90 : public BT::SyncActionNode
{
public:
    explicit TurnRight90(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Turn_right_90_deg" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Detect_gate_at_front_of_sub (Condition Node) 
BT::NodeStatus Detect_gate_at_front_of_sub()
{
    std::this_thread::sleep_for(3s);
    cout << "Detect_gate_at_front_of_sub" << endl; 
    return BT::NodeStatus::SUCCESS;
}
 
// Turn_right_until_parallel_with_Path_and_facing_away_from_the_end (Action Node)
class Turn_right_until_parallel_with_Path_and_facing_away_from_the_end : public BT::SyncActionNode
{
public:
    explicit Turn_right_until_parallel_with_Path_and_facing_away_from_the_end(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Turn_right_until_parallel_with_Path_and_facing_away_from_the_end" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Reposition_sub_to_gate_left_entrance (Action Node)
class Reposition_sub_to_gate_left_entrance : public BT::SyncActionNode
{
public:
    explicit Reposition_sub_to_gate_left_entrance(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Reposition_sub_to_gate_left_entrance" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Reposition_sub_to_gate_right_entrance (Action Node)
class Reposition_sub_to_gate_right_entrance : public BT::SyncActionNode
{
public:
    explicit Reposition_sub_to_gate_right_entrance(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Reposition_sub_to_gate_right_entrance" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Set_preferred_side_to_L (PORT ASSOCIATED -- ACTION NODE)
class Set_preferred_side_to_L : public BT::SyncActionNode
{
public:
    explicit Set_preferred_side_to_L(const std::string &name, const BT::NodeConfig &config)
     : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<string>("preferred_side") };
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Set_preferred_side_to_L" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Set_preferred_side_to_R (PORT ASSOCIATED -- ACTION NODE)
class Set_preferred_side_to_R : public BT::SyncActionNode
{
public:
    explicit Set_preferred_side_to_R(const std::string &name, const BT::NodeConfig &config)
     : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<string>("preferred_side") };
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Set_preferred_side_to_R" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Detect_preferred_animal_left_of_center (PORT ASSOCIATED -- CONDITION NODE?)
class Detect_preferred_animal_left_of_center : public BT::SyncActionNode
{
public:
    explicit Detect_preferred_animal_left_of_center(const std::string &name, const BT::NodeConfig &config)
     : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<string>("preferred_animal") };
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Detect_preferred_animal_left_of_center" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Move_with_style_through_gate (Action Node)
class Move_with_style_through_gate : public BT::SyncActionNode
{
public:
    explicit Move_with_style_through_gate(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_with_style_through_gate" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Move_in_the_most_boring_way_possible_through_the_gate (Action Node)
class Move_in_the_most_boring_way_possible_through_the_gate : public BT::SyncActionNode
{
public:
    explicit Move_in_the_most_boring_way_possible_through_the_gate(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_in_the_most_boring_way_possible_through_the_gate" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};
 
// Move_until_the_other_end_of_the_path (Action Node)
class Move_until_the_other_end_of_the_path : public BT::SyncActionNode
{
public:
    explicit Move_until_the_other_end_of_the_path(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_until_the_other_end_of_the_path" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// preferred_side_is_L (Condition Node)
BT::NodeStatus preferred_side_is_L(BT::TreeNode &self) 
{
    auto side = self.getInput<std::string>("preferred_side");
    if (side.value() == "L")
    {
        cout << "Preferred Side is L" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
    cout << "Preferred Side is NOT L" << endl; 
    return BT::NodeStatus::FAILURE;
}


// Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right
class Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right : public BT::SyncActionNode
{
public:
    explicit Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Move_past_PVC_posts
class Move_past_PVC_posts : public BT::SyncActionNode
{
public:
    explicit Move_past_PVC_posts(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_past_PVC_posts" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right
class Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right : public BT::SyncActionNode
{
public:
    explicit Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Move_sub_until_aligned_with_preferred_animal_on_bin 
class Move_sub_until_aligned_with_preferred_animal_on_bin : public BT::SyncActionNode
{
public:
    explicit Move_sub_until_aligned_with_preferred_animal_on_bin(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_sub_until_aligned_with_preferred_animal_on_bin" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Drop_marker
class Drop_marker : public BT::SyncActionNode
{
public:
    explicit Drop_marker(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Drop_marker" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Find_task_location
class Find_task_location : public BT::SyncActionNode
{
public:
    explicit Find_task_location(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Find_task_location" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Center_on_board_on_the_z_axis_and_yaw
class Center_on_board_on_the_z_axis_and_yaw : public BT::SyncActionNode
{
public:
    explicit Center_on_board_on_the_z_axis_and_yaw(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Center_on_board_on_the_z_axis_and_yaw" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal
class Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal : public BT::SyncActionNode
{
public:
    explicit Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Detect_preferred_animal_at_front_of_sub_roughly
BT::NodeStatus Detect_preferred_animal_at_front_of_sub_roughly() 
{
    std::this_thread::sleep_for(3s);
    cout << "Detect_preferred_animal_at_front_of_sub_roughly" << endl; 
    return BT::NodeStatus::SUCCESS;
}

// Center_sub_perpendicular_to_preferred_animal
class Center_sub_perpendicular_to_preferred_animal : public BT::SyncActionNode
{
public:
    explicit Center_sub_perpendicular_to_preferred_animal(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Center_sub_perpendicular_to_preferred_animal" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Grab_trash_with_claw
class Grab_trash_with_claw : public BT::SyncActionNode
{
public:
    explicit Grab_trash_with_claw(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Grab_trash_with_claw" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Resurface
class Resurface : public BT::SyncActionNode
{
public:
    explicit Resurface(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Resurface" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Move_and_place_trash_in_corresponding_basket
class Move_and_place_trash_in_corresponding_basket : public BT::SyncActionNode
{
public:
    explicit Move_and_place_trash_in_corresponding_basket(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_and_place_trash_in_corresponding_basket" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Move_back_facing_initial_direction
class Move_back_facing_initial_direction : public BT::SyncActionNode
{
public:
    explicit Move_back_facing_initial_direction(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_back_facing_initial_direction" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Move_to_depth_taken_for_Navigating_the_Channel
class Move_to_depth_taken_for_Navigating_the_Channel : public BT::SyncActionNode
{
public:
    explicit Move_to_depth_taken_for_Navigating_the_Channel(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Move_to_depth_taken_for_Navigating_the_Channel" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Turn_right_180_deg
class Turn_right_180_deg : public BT::SyncActionNode
{
public:
    explicit Turn_right_180_deg(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Turn_right_180_deg" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

// Make_it_back_through_gate
class Make_it_back_through_gate : public BT::SyncActionNode
{
public:
    explicit Make_it_back_through_gate(const std::string &name) : BT::SyncActionNode(name, {})
    {
    }
 
    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        cout << "Make_it_back_through_gate" << endl; 
        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    auto blackboard = BT::Blackboard::create(); 
    blackboard->set("detections", "");

    BT::BehaviorTreeFactory factory; 
    
    // Heading Out Nodes
    factory.registerSimpleCondition("Detect_gate_at_front_of_sub", std::bind(Detect_gate_at_front_of_sub));
    BT::PortsList detections = {BT::InputPort<string>("detections")};
    factory.registerNodeType<Submerge>("Submerge");
    factory.registerNodeType<Center_sub_perpendicular_to_gate>("Center_sub_perpendicular_to_gate");
    factory.registerNodeType<TurnRight90>("Turn_right_90_deg");
 
    // Collecting Data Nodes
    factory.registerNodeType<Detect_preferred_animal_left_of_center>("Detect_preferred_animal_left_of_center");
    factory.registerNodeType<Reposition_sub_to_gate_left_entrance>("Reposition_sub_to_gate_left_entrance");
    factory.registerNodeType<Set_preferred_side_to_L>("Set_preferred_side_to_L");
    factory.registerNodeType<Reposition_sub_to_gate_right_entrance>("Reposition_sub_to_gate_right_entrance");
    factory.registerNodeType<Set_preferred_side_to_R>("Set_preferred_side_to_R");
    factory.registerNodeType<Move_with_style_through_gate>("Move_with_style_through_gate");
    factory.registerNodeType<Move_in_the_most_boring_way_possible_through_the_gate>("Move_in_the_most_boring_way_possible_through_the_gate");
    factory.registerNodeType<Turn_right_until_parallel_with_Path_and_facing_away_from_the_end>("Turn_right_until_parallel_with_Path_and_facing_away_from_the_end");
    factory.registerNodeType<Move_until_the_other_end_of_the_path>("Move_until_the_other_end_of_the_path");

    // Navigate the Channel Nodes
    BT::PortsList preferred_side_is_L_ports = {BT::InputPort<string>("preferred_side")};
    factory.registerSimpleCondition("preferred_side_is_L", preferred_side_is_L, preferred_side_is_L_ports);
    factory.registerNodeType<Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right>("Center_sub_to_face_midpoint_between_white_PVC_on_the_left_and_red_PVC_on_the_right");
    factory.registerNodeType<Move_past_PVC_posts>("Move_past_PVC_posts");
    factory.registerNodeType<Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right>("Center_sub_to_face_midpoint_between_red_PVC_on_the_left_and_white_PVC_on_the_right");

    // Drop a BRUVs Nodes 
    factory.registerNodeType<Move_sub_until_aligned_with_preferred_animal_on_bin>("Move_sub_until_aligned_with_preferred_animal_on_bin");
    factory.registerNodeType<Drop_marker>("Drop_marker");

    // Tagging Nodes 
    factory.registerNodeType<Find_task_location>("Find_task_location");
    factory.registerNodeType<Center_on_board_on_the_z_axis_and_yaw>("Center_on_board_on_the_z_axis_and_yaw");
    factory.registerNodeType<Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal>("Detect_and_fire_torpedoes_at_opening_closest_to_preferred_animal");

    // Ocean Cleanup Nodes 
    factory.registerSimpleCondition("Detect_preferred_animal_at_front_of_sub_roughly", std::bind(Detect_preferred_animal_at_front_of_sub_roughly));
    factory.registerNodeType<Center_sub_perpendicular_to_preferred_animal>("Center_sub_perpendicular_to_preferred_animal");
    factory.registerNodeType<Grab_trash_with_claw>("Grab_trash_with_claw");
    factory.registerNodeType<Resurface>("Resurface");
    factory.registerNodeType<Move_and_place_trash_in_corresponding_basket>("Move_and_place_trash_in_corresponding_basket");
    factory.registerNodeType<Move_back_facing_initial_direction>("Move_back_facing_initial_direction");

    // Return Home Nodes 
    factory.registerNodeType<Move_to_depth_taken_for_Navigating_the_Channel>("Move_to_depth_taken_for_Navigating_the_Channel");
    factory.registerNodeType<Turn_right_180_deg>("Turn_right_180_deg");
    factory.registerNodeType<Make_it_back_through_gate>("Make_it_back_through_gate");

    // Register main tree and subtrees 
    
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/main.xml"); // Main tree
    
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/heading_out.xml"); // Heading Out (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/collecting_data.xml"); // Collecting Data (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/folllow_path.xml"); // Follow Path (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/navigate_channel.xml"); // Navigate the Channel (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/drop_bruvs.xml"); // Drop a BRUVS (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/tagging.xml"); // Tagging (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/ocean_cleanup.xml"); // Ocean Cleanup (Subtree)
    factory.registerBehaviorTreeFromFile("/home/yirehban/ros2_ws/src/mission/bt_xml/return_home.xml"); // Return Home (Subtree)

    // Spin ROS node in separate tree
    auto mission = std::make_shared<DetectionNodeConfig>(blackboard);
    std::thread ros_spin_thread([&]() { rclcpp::spin(mission); });

    // Create the main tree
    auto main_tree = factory.createTree("SHRUB (Software for Handling and Regulating Underwater Behavior)", blackboard);

    // Tick the tree in the main thread
    main_tree.tickWhileRunning();
    if (!rclcpp::ok())
    { 
        rclcpp::shutdown();
    }
    ros_spin_thread.join();

    return 0;
    
}

