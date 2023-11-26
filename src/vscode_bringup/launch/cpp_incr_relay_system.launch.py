import launch


def generate_launch_description():
    from launch import LaunchDescription
    from launch_ros.actions import Node

    incrementer_node = Node(
        package="vscode_cpp",
        executable="incrementer_node",
        name="incrementer",
        output="screen",
        # parameters=[{"step_size": 2}],
        # prefix=["gdbserver localhost:3000"],
    )

    delayed_relay_node = Node(
        package="vscode_py",
        executable="delayed_relay_node",
        name="delayed_relay",
        output="screen",
        remappings=[
            ("number", "incremented_number"),
            ("delayed_number", "number"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(incrementer_node)
    ld.add_action(delayed_relay_node)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
