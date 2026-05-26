cmake_minimum_required(VERSION 3.8)
project(rgt_manager)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  src/rgt_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rgt_manager
)
install(PROGRAMS
  scripts/spacemouse.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spacemouse
)
install(PROGRAMS
  scripts/find_tray_reference.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME find_tray_reference
)
install(PROGRAMS
  scripts/test_tray_reference.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_tray_reference
)

install(DIRECTORY
  config
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
