import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _box_sdf(name: str, size_xyz: tuple, rgba: tuple, static: bool = False, physical: bool = True) -> str:
    sx, sy, sz = size_xyz
    r, g, b, a = rgba
    
    collision_tag = f"""
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>""" if physical else ""
    
    gravity_tag = "<gravity>1</gravity>" if physical else "<gravity>0</gravity>"

    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <static>{'true' if static else 'false'}</static>
    <link name="link">
      {gravity_tag}
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
      {collision_tag}
    </link>
  </model>
</sdf>"""

def _spawn_box_classic(name: str, x: float, y: float, z: float,
                        size_xyz: tuple, rgba: tuple, static: bool = False, physical: bool = True) -> Node:
    sdf_content = _box_sdf(name, size_xyz, rgba, static=static, physical=physical)
    tmp_path = f"/tmp/{name}.sdf"
    with open(tmp_path, "w") as f:
        f.write(sdf_content)
    return Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", name, "-file", tmp_path,
                   "-x", str(x), "-y", str(y), "-z", str(z)],
        output="screen",
    )

def generate_launch_description():
    ur_sim_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    pkg_share = get_package_share_directory('ur3_llm_control')

    ur_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_sim_pkg_dir, 'launch', 'ur_sim_moveit.launch.py')
        ),
        launch_arguments={'ur_type': 'ur3', 'world': os.path.join(pkg_share, 'worlds', 'ur_scene.world')}.items()
    )

    OBJECT_COLORS = {
        "red_cube": (1.0, 0.0, 0.0, 1.0),
        "yellow_cube": (1.0, 1.0, 0.0, 1.0),
        "blue_cube": (0.0, 0.0, 1.0, 1.0),
    }
    ZONE_COLORS = {
        "zone_a": (1.0, 0.6, 0.6, 0.7),
        "zone_b": (1.0, 1.0, 0.6, 0.7),
        "zone_c": (0.6, 0.6, 1.0, 0.7),
        "temp_zone": (0.7, 0.7, 0.7, 0.6),
    }

    spawn_actions = []
    
   
    spawn_actions.append(_spawn_box_classic(
        "work_table", 0.35, 0.0, -0.025, (1.0, 0.8, 0.05), (0.55, 0.35, 0.2, 1.0),
        static=True, physical=True
    ))

    
    objects_data = {
        "red_cube":    {"x": 0.28, "y":  0.15, "z": 0.02},
        "yellow_cube": {"x": 0.28, "y":  0.00, "z": 0.02},
        "blue_cube":   {"x": 0.28, "y": -0.15, "z": 0.02},
    }
    for name, xyz in objects_data.items():
        rgba = OBJECT_COLORS.get(name, (0.7, 0.7, 0.7, 1.0))
        spawn_actions.append(_spawn_box_classic(
            name, xyz["x"], xyz["y"], xyz["z"], (0.04, 0.04, 0.04), rgba,
            static=False, physical=False
        ))

    
    zones_data = {
        "zone_a":    {"x": 0.40, "y":  0.15, "z": 0.005},
        "zone_b":    {"x": 0.40, "y":  0.00, "z": 0.005},
        "zone_c":    {"x": 0.40, "y": -0.15, "z": 0.005},
        "temp_zone": {"x": 0.34, "y":  0.30, "z": 0.005},
    }
    for name, xyz in zones_data.items():
        rgba = ZONE_COLORS.get(name, (0.8, 0.8, 0.8, 0.6))
        spawn_actions.append(_spawn_box_classic(
            name, xyz["x"], xyz["y"], xyz["z"], (0.08, 0.08, 0.005), rgba,
            static=True, physical=True
        ))

    scene_node = Node(
        package="ur3_llm_control",
        executable="scene_publisher",
        name="scene_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        ur_sim_launch,
        TimerAction(period=6.0, actions=spawn_actions),
        TimerAction(period=10.0, actions=[scene_node]),
    ])
