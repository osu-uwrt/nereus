# Reporting security issues

Report vulnerabilities privately to the OSU Underwater Robotics Team maintainers rather than in a
public issue, and leave robot network details and credentials out of any report.

The simulator and viewer trust the packs and the ROS graph they are given; there is no sandbox.
When the viewer drives a real robot it sends real motion and actuator commands. It does not replace
the robot's hardware kill switch or other safety systems.
