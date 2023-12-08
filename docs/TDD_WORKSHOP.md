## TDD Workshop

#### Introduction

- Go over the Red, Green, Refactor cycle
- Unit tests, integration tests and system tests. What is the difference?

(Unit tests focus on individual code units, integration tests examine component interactions, and system tests assess the entire software system.)

#### Tooling used in this workshop

- Docker
- VSCode, Dev Containers
- Colcon (build + test)
- Trunk Check (linting + formatting)
- Github Copilot
- Gdbserver
- VSCode, Python Test Explorer

#### Mission 1: Explore

- Look through the workspace to find out what it does. (Hint: find and use the tests as documentation).
- Make sure to take a look at the system test.

#### Mission 2: Poke it until it breaks

- Try to get specific tests to fail by changing the code.

#### Mission 3: Try debugging tools

- Try using VSCodes debugging tools for both python and cpp. Set breakpoints etc.
- Make sure to try to debug nodes that were started in a launchfile.

#### Mission 4: Add new funtions to one of the exsisting ROS2 nodes

- Start with a python node:
- Decide on what new functionality to add.
- Write a test that describes the new functionality. Make sure it fails (Red).
- Write minimal code to make the test and all the old test pass (Green).
- (Refactor) the code and make sure all tests are still green.

#### Mission 5: Make a new ROS2 node from scratch using TDD

- Decide what the node should do
- Set up the testing skeleton
- Make the code skeleton for the node
- Write an initial failing test
- Write code to make the test pass
- Refactor if needed
- Repeat

#### Mission 6: Linting

- Fix project linting issues

#### (Mission 7: Convert the delayed_relay node to cpp using TDD)
