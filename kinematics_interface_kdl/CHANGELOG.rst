^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kinematics_interface_kdl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [kilted] Update deprecated call to ament_target_dependencies (backport `#138 <https://github.com/ros-controls/kinematics_interface/issues/138>`_) (`#142 <https://github.com/ros-controls/kinematics_interface/issues/142>`_)
* Use ros2_control_cmake (backport `#118 <https://github.com/ros-controls/kinematics_interface/issues/118>`_) (`#119 <https://github.com/ros-controls/kinematics_interface/issues/119>`_)
* Contributors: mergify[bot]

1.3.0 (2025-03-11)
------------------
* Remove visibility boilerplate code (`#101 <https://github.com/ros-controls/kinematics_interface/issues/101>`_)
* Calculate Jacobian Inverse (`#92 <https://github.com/ros-controls/kinematics_interface/issues/92>`_)
* Contributors: Christoph Fröhlich, francesco-donofrio

1.2.1 (2024-11-11)
------------------
* Remove ns from robot_description parameter (`#91 <https://github.com/ros-controls/kinematics_interface/issues/91>`_)
* Contributors: Christoph Fröhlich

1.2.0 (2024-11-05)
------------------
* API changes to support robot description (`#83 <https://github.com/ros-controls/kinematics_interface/issues/83>`_)
* Contributors: Dr. Denis

1.1.0 (2024-05-20)
------------------
* Read base parameter in initialize function (`#73 <https://github.com/ros-controls/kinematics_interface/issues/73>`_)
* Use CMake targets for eigen (`#50 <https://github.com/ros-controls/kinematics_interface/issues/50>`_)
* Contributors: Bence Magyar, Christoph Fröhlich

1.0.0 (2023-11-16)
------------------
* 🤔 Remove compile warnings and unify for-loop syntax. (`#15 <https://github.com/ros-controls/kinematics_interface/issues/15>`_)
* Contributors: Dr. Denis, Bence Magyar, Paul Gesel

0.1.0 (2023-01-27)
------------------
* Fix overriding of install (`#13 <https://github.com/ros-controls/kinematics_interface/issues/13>`_)
* Contributors: Tyler Weaver

0.0.2 (2022-10-21)
------------------

0.0.1 (2022-10-11)
------------------
* Enable interface to use Eigen per default (`#6 <https://github.com/ros-controls/kinematics_interface/issues/6>`_)
* Add CI setup to the repository. (`#8 <https://github.com/ros-controls/kinematics_interface/issues/8>`_)
* Remove duplicate of Eigen dependency.
* Update CMakeFiles, correct deps and formatting. (`#10 <https://github.com/ros-controls/kinematics_interface/issues/10>`_)
* add pre-commit to repo (`#7 <https://github.com/ros-controls/kinematics_interface/issues/7>`_)
  add pre-commit
* Fix failing KDL test (`#5 <https://github.com/ros-controls/kinematics_interface/issues/5>`_)
  * add verification for all inputs
  * add underscore to end_effector
  * add test to ensure inverse then forward calculation is approximately unit
* Fix more indentations and alphabetize
* Update dependencies and maintainers
* Kinematics interface with KDL implementation
* change directory name and use header only interface
* Contributors: Paul Gesel, Andy Zelenak, Bence Magyar, Denis Štogl
