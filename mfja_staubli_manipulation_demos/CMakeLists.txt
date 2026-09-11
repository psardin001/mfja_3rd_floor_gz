cmake_minimum_required(VERSION 3.28.0)
project(mfja_staubli_manipulation_demos)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs hpp launch scripts
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
