using Documenter
makedocs(
doctest=false, clean=true,
format =:html,
authors="Huckleberry Febbo",
assets = ["assets/style.css"],
sitename="MAVs",
pages = [
    "Home" => "index.md",
    "Usage" => [
        "Basic usage" => "tutorials/basic.md",
        "Developing MAVs" => "tutorials/developer.md",
    ],
    "Sensing" => [
        "vehicle_description" => "packages/model/gazebo/vehicle_description/main.md",
        "contact_sensor" => "packages/model/gazebo/contact_sensor/main.md",
    ],
    "Detection" => [
        "obstacle_detector" => "packages/computing/perception/obstacle_detector/main.md",
    ],
    "Vehicle model" => [
        "ros_chrono" => "packages/model/chrono/ros_chrono/main.md",
    ],
    "Planning" => [
        "nloptcontrol_planner" => "packages/computing/planning/nloptcontrol_planner/main.md",
        "mavs_ros_planner" => "packages/computing/planning/mavs_ros_planner/main.md",
    ],
    "Controller" => [
        "path_follower" => "packages/computing/controller/path_follower/main.md",
        "trajectory_follower" => "packages/computing/controller/trajectory_follower/main.md",
    ],
    "Miscellaneous" => [
        "system_shutdown" => "packages/system/system_shutdown.md",
        "data_logging" => "packages/system/data_logging.md",
        "chrono_position_broadcaster" => "packages/model/chrono/chrono_position_broadcaster/main.md",
        "point_cloud_converter" => "packages/computing/perception/point_cloud_converter.md",
    ],
    "System demos" => [
        "demoA" => "packages/system/demos/demoA.md",
        "demoB" => "packages/system/demos/demoB.md",
        "demoC" => "packages/system/demos/demoC.md",
        "demoD" => "packages/system/demos/demoD.md",
        "demoE" => "packages/system/demos/demoE.md",
        "demoF" => "packages/system/demos/demoF.md",
        "demoG" => "packages/system/demos/demoG.md",
        "demoH" => "packages/system/demos/demoH.md",
        "demoI" => "packages/system/demos/demoI.md",
    ],
    "Docker" => [
        "Workflow" => "docker/workflow.md",
        "Notes" => "docker/notes.md",
    ],
    "Testing"  => "testing/main.md",
    "Troubleshooting"  => "issues/main.md",
]
)

deploydocs(
    repo="github.com/JuliaMPC/MAVs.git",
)
