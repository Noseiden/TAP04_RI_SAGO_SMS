// Parte 5: Jacobiano de MoveIt2 con RobotState::getJacobian() para una lista de configuraciones.
// Entrada: CSV con 6 angulos por fila (joint_1..joint_6, rad).
// Salida : CSV con las 36 entradas del Jacobiano 6x6 por fila (fila por fila: [vx vy vz wx wy wz] x [q1..q6]),
//          punto de referencia = origen de tool0, expresado en el marco del modelo (world = base_link).
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "jacobiano_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const std::string entrada = node->get_parameter("entrada").as_string();
  const std::string salida = node->get_parameter("salida").as_string();

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const moveit::core::RobotModelPtr& model = loader.getModel();
  moveit::core::RobotState state(model);
  const moveit::core::JointModelGroup* grupo = model->getJointModelGroup("manipulator");
  const moveit::core::LinkModel* tool0 = model->getLinkModel("tool0");

  std::ostringstream orden;
  for (const auto& n : grupo->getVariableNames())
    orden << n << " ";
  RCLCPP_INFO(node->get_logger(), "Grupo 'manipulator', orden de variables: %s| marco del modelo: %s",
              orden.str().c_str(), model->getModelFrame().c_str());

  std::ifstream fin(entrada);
  std::ofstream fout(salida);
  fout << std::setprecision(12);
  std::string linea;
  size_t n = 0;
  Eigen::MatrixXd J;
  while (std::getline(fin, linea))
  {
    std::stringstream ss(linea);
    std::vector<double> q;
    for (std::string c; std::getline(ss, c, ',');)
      q.push_back(std::stod(c));
    if (q.size() != 6)
      continue;
    state.setJointGroupPositions(grupo, q);
    state.updateLinkTransforms();
    state.getJacobian(grupo, tool0, Eigen::Vector3d::Zero(), J);
    for (int i = 0; i < 6; ++i)
      for (int k = 0; k < 6; ++k)
        fout << J(i, k) << ((i == 5 && k == 5) ? "\n" : ",");
    ++n;
  }
  RCLCPP_INFO(node->get_logger(), "%zu Jacobianos escritos en %s", n, salida.c_str());
  rclcpp::shutdown();
  return 0;
}
