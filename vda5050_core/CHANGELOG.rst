^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vda5050_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix(client): transform node positions if a transformation is available (`#80 <https://github.com/ros-industrial/vda5050_core/issues/80>`_)
* feat(client): provide initial adapter interface along with rmf migration guide (`#71 <https://github.com/ros-industrial/vda5050_core/issues/71>`_)
* feat(master): add order lifecycle, stitcher, instant actions, mode handling, and pose_view (`#69 <https://github.com/ros-industrial/vda5050_core/issues/69>`_)
* feat: master-side event detectors and heartbeat hardening (`#58 <https://github.com/ros-industrial/vda5050_core/issues/58>`_)
* refactor(layout): use nested namespaces in headers for C++14 compat (`#78 <https://github.com/ros-industrial/vda5050_core/issues/78>`_)
* feat(master): add traversability, action-conflict, factsheet, and mode validators (`#56 <https://github.com/ros-industrial/vda5050_core/issues/56>`_)
* feat(master): add schema and pre-send validators (`#55 <https://github.com/ros-industrial/vda5050_core/issues/55>`_)
* Add unsubscribe_all() to ProtocolAdapter (`#74 <https://github.com/ros-industrial/vda5050_core/issues/74>`_)
* feat(client): add order validator and acceptance strategy (`#63 <https://github.com/ros-industrial/vda5050_core/issues/63>`_)
* feat(layout): add LIF topology graph and JSON loader (`#54 <https://github.com/ros-industrial/vda5050_core/issues/54>`_)
* refactor(client): move context and resource implementations to source files (`#66 <https://github.com/ros-industrial/vda5050_core/issues/66>`_)
* feat(client): add AGVContext and execution tracking resource (`#62 <https://github.com/ros-industrial/vda5050_core/issues/62>`_)
* feat(master): change interface_name to be configurable in vda5050_core::master (`#59 <https://github.com/ros-industrial/vda5050_core/issues/59>`_)
* feat(client): add concrete execution data types (`#53 <https://github.com/ros-industrial/vda5050_core/issues/53>`_)
* feat: add method to suspend engine indefinitely (`#50 <https://github.com/ros-industrial/vda5050_core/issues/50>`_)
* feat(mqtt)!: changes to mqtt client and protocol adapter  (`#48 <https://github.com/ros-industrial/vda5050_core/issues/48>`_)
* refactor: reorganize the package to build using plain C++ instructions (`#44 <https://github.com/ros-industrial/vda5050_core/issues/44>`_)
* chore: add license and change emails in pkg xml files (`#43 <https://github.com/ros-industrial/vda5050_core/issues/43>`_)
* feat(ci): add asan/tsan to ci (`#35 <https://github.com/ros-industrial/vda5050_core/issues/35>`_)
* feat: order validator changes (`#31 <https://github.com/ros-industrial/vda5050_core/issues/31>`_)
* Feat (VDA5050 master) : Initial commit of VDA5050 Master Codebase (`#20 <https://github.com/ros-industrial/vda5050_core/issues/20>`_)
* feat(humble): add support for humble (`#27 <https://github.com/ros-industrial/vda5050_core/issues/27>`_)
* feat(ci): run mosquitto locally in workflows to provide broker for tests (`#25 <https://github.com/ros-industrial/vda5050_core/issues/25>`_)
* feat(mqtt): update mqtt interface with new methods (`#19 <https://github.com/ros-industrial/vda5050_core/issues/19>`_)
* Feat(state-manager): add StateManager (`#14 <https://github.com/ros-industrial/vda5050_core/issues/14>`_)
* Feat(mqtt_client) : Add Unsubscribe method (`#17 <https://github.com/ros-industrial/vda5050_core/issues/17>`_)
* feat(types): create C++ structs for VDA5050 2.0.0 spec (`#6 <https://github.com/ros-industrial/vda5050_core/issues/6>`_)
* feat(mqtt): add MQTT client interface with Paho as an default implementation (`#1 <https://github.com/ros-industrial/vda5050_core/issues/1>`_)
* Contributors: Chen Bainian, Eileen Teoh Yan Zhen, John Arman Abogado, Leah, Saurabh Kamat, Lim Yew Hao, Glenn Tan
