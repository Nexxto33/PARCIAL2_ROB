#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>





void move_to_pose(
  moveit::planning_interface::MoveGroupInterface &move_group,
  const geometry_msgs::msg::Pose &target_pose,
  rclcpp::Logger logger)
{
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, " Ejecutando ");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "NO se pudo alcanzar el objetivo.");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "menu_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto logger = rclcpp::get_logger("menu_moveit");

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = -0.58;
  pose1.position.y = 0.2;
  pose1.position.z = 0.5;

  geometry_msgs::msg::Pose pose2;
  pose2.orientation.w = 0.7;
  pose2.position.x = 0.9;
  pose2.position.y = 0.4;
  pose2.position.z = 0.1;

  geometry_msgs::msg::Pose pose3;
  pose3.orientation.w = 0.0;
  pose3.position.x = 1.5;
  pose3.position.y = 1.1;
  pose3.position.z = 0.9;

  int choice;
  do {
    std::cout << "\n=== ELIJA UNA OPCION ===\n";
    std::cout << "1) Posición  x = -0.58, y = 0.2, z = 0.5 ,  w = 1.0  :  \n";
    std::cout << "2) Posición  x = 0.9, y = 0.4, z = 0.1 , w = 0.7 :  \n";
    std::cout << "3) Posición  x = 1.5, y = 1.1, z = 0.9 , w = 0.0 : \n";
    std::cout << "4) Ingresar nueva posición \n";
    std::cout << "5) Salir \n";
    std::cout << "Ingrese su opcion seleccionada : ";
    std::cin >> choice;

    geometry_msgs::msg::Pose custom_pose;

    switch (choice) {
      case 1:
        RCLCPP_INFO(logger, "Ejecutando Posición 1 ");
        move_to_pose(move_group, pose1, logger);
        break;
      case 2:
        RCLCPP_INFO(logger, "Ejecutando Posición 2 ");
        move_to_pose(move_group, pose2, logger);
        break;
      case 3:
        RCLCPP_INFO(logger, "Ejecutando Posición 3 ");
        move_to_pose(move_group, pose3, logger);
        break;
      case 4:
        std::cout << "Ingrese X: "; std::cin >> custom_pose.position.x;
        std::cout << "Ingrese Y: "; std::cin >> custom_pose.position.y;
        std::cout << "Ingrese Z: "; std::cin >> custom_pose.position.z;
        std::cout << "Ingrese W (orientación): "; std::cin >> custom_pose.orientation.w;
        custom_pose.orientation.x = 0.0;
        custom_pose.orientation.y = 0.0;
        custom_pose.orientation.z = 0.0;
        RCLCPP_INFO(logger, "Moviendo a posición solicitada ");
        move_to_pose(move_group, custom_pose, logger);
        break;
      case 5:
        RCLCPP_INFO(logger, "Saliendo del programa ");
        break;
      default:
        std::cout << "Opción no válida, intente de nuevo.\n";
    }
  } while (choice != 5);

  rclcpp::shutdown();
  return 0;
}
