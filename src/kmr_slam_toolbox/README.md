# 1 - Install notes:
Errors occur due to Qt5 package being called without any dependencies. This is a problem with the current "rviz_default_plugins" package. To fix this, you need to edit the file in

~/<rviz_plugins_install_folder>/install/rviz_default_plugins/share/rviz_default_plugins/cmake/ament_cmake_export_dependencies-extras.cmake

In line 3 witin set(_exported_dependencies), you will need to change "Qt5" into "Qt5Core;Qt5Widgets".
