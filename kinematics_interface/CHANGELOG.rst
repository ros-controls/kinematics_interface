^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kinematics_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove remnant of visibility control (`#170 <https://github.com/ros-controls/kinematics_interface/issues/170>`_)
* Pass Eigen3 to ament_export_dependencies (`#165 <https://github.com/ros-controls/kinematics_interface/issues/165>`_)
* Contributors: Christoph Froehlich, Christoph Fröhlich, Silvio Traversaro

2.1.0 (2025-05-23)
------------------
* [kilted] Update deprecated call to ament_target_dependencies (`#138 <https://github.com/ros-controls/kinematics_interface/issues/138>`_)
* Use ros2_control_cmake (`#118 <https://github.com/ros-controls/kinematics_interface/issues/118>`_)
* Contributors: Christoph Fröhlich, David V. Lu!!

2.0.0 (2025-03-11)
------------------
* Add methods for computing frame differences (`#93 <https://github.com/ros-controls/kinematics_interface/issues/93>`_)
* Contributors: francesco-donofrio

1.3.0 (2025-03-11)
------------------
* Remove visibility boilerplate code (`#101 <https://github.com/ros-controls/kinematics_interface/issues/101>`_)
* Calculate Jacobian Inverse (`#92 <https://github.com/ros-controls/kinematics_interface/issues/92>`_)
* Contributors: Christoph Fröhlich, francesco-donofrio

1.2.1 (2024-11-11)
------------------

1.2.0 (2024-11-05)
------------------
* API changes to support robot description (`#83 <https://github.com/ros-controls/kinematics_interface/issues/83>`_)
* Contributors: Dr. Denis

1.1.0 (2024-05-20)
------------------
* Use CMake targets for eigen (`#50 <https://github.com/ros-controls/kinematics_interface/issues/50>`_)
* Contributors: Christoph Fröhlich

1.0.0 (2023-11-16)
------------------
* Use a dynamic library instead of header-only (`#21 <https://github.com/ros-controls/kinematics_interface/issues/21>`_)
* Contributors: Thibault Poignonec

0.1.0 (2023-01-27)
------------------
* Fix overriding of install (`#13 <https://github.com/ros-controls/kinematics_interface/issues/13>`_)
* [CI] Fix and update formatting workflow and hooks (`#14 <https://github.com/ros-controls/kinematics_interface/issues/14>`_)
* Contributors: Dr. Denis, Tyler Weaver

0.0.2 (2022-10-21)
------------------

0.0.1 (2022-10-11)
------------------
* Make formatting of comments unified and nice. (`#11 <https://github.com/ros-controls/kinematics_interface/issues/11>`_)
* Enable interface to use Eigen per default (`#6 <https://github.com/ros-controls/kinematics_interface/issues/6>`_)
* Add CI setup to the repository. (`#8 <https://github.com/ros-controls/kinematics_interface/issues/8>`_)
* Update CMakeFiles, correct deps and formatting. (`#10 <https://github.com/ros-controls/kinematics_interface/issues/10>`_)
* add pre-commit to repo (`#7 <https://github.com/ros-controls/kinematics_interface/issues/7>`_)
* Fix more indentations and alphabetize
* Update dependencies and maintainers
* Kinematics interface with KDL implementation
* change directory name and use header only interface
* changed plugin name to kinematics_interface_kdl/KDLKinematics
* add calculate_jacobian as new interface method
* added argument to specify which link the Jacobian is calculated with respect to
* made interface stateless, changed method name, fixed small bugs
* updated README and cleaned up code
* added KDL plugin for kinematics interface
* Contributors: Paul Gesel, Andy Zelenak, Bence Magyar, Denis Štogl
