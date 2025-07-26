#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/rotate.hpp"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/move_to.hpp"
#include "/home/yirehban/ros2_ws/src/mission/include/mission/detect_object.hpp"

using namespace std; 

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<Rotate>("Rotate");
    factory.registerNodeType<MoveTo>("MoveTo");
    factory.registerNodeType<DetectObject>("DetectObject");

    auto tree = factory.createTreeFromFile("src/mission/bt_xml/mission1.xml");

    BT::StdCoutLogger logger(tree);
    tree.rootNode()->printTreeRecursively();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        tree.tickRoot();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

