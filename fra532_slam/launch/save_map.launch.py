from pathlib import Path
from launch_ros.actions.node import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable

def get_next_map_prefix(maps_dir: Path, base_name: str = "map"):
    """
    Check if a map file already exists (by looking for a .yaml file).
    If no map file is found, return the base file name.
    If a file exists, loop and append an incrementing number until an available file name is found.
    """
    candidate = maps_dir / base_name
    if not (candidate.with_suffix('.yaml')).exists():
        return str(candidate)
    counter = 1
    while (maps_dir / f"{base_name}_{counter}").with_suffix('.yaml').exists():
        counter += 1
    return str(maps_dir / f"{base_name}_{counter}")

def generate_launch_description():
    # Define the maps directory in the user's home folder
    maps_dir = Path.home() / "maps"
    # Create the maps directory if it does not exist
    maps_dir.mkdir(parents=True, exist_ok=True)
    
    # Get the next available map prefix to avoid overwriting an existing file
    next_map_prefix = get_next_map_prefix(maps_dir)

    # Launch an ExecuteProcess to create the maps directory (for compatibility with other launch actions)
    mkdir_maps = ExecuteProcess(
        cmd=[
            FindExecutable(name='mkdir'),
            ' -p ',
            str(maps_dir)
        ],
        shell=True
    )

    # Define the map_saver_cli node with the argument for the output file prefix
    map_saver_cli = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        arguments=['-f', next_map_prefix],
        parameters=[{'save_map_timeout': 10000.0}]
    )

    # Register an event handler to launch map_saver_cli after mkdir_maps exits
    delay_map_saver_cli_after_mkdir_maps = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mkdir_maps,
            on_exit=[map_saver_cli]
        )
    )

    return LaunchDescription([
        mkdir_maps,
        delay_map_saver_cli_after_mkdir_maps
    ])