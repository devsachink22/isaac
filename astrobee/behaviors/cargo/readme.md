\page cargo Cargo Behavior

This directory provides the cargo_tool

# Using the cargo tool

This tool is used to initiate pickup and drop cargo actions.

To run the tool:
	
	rosrun cargo cargo_tool -$ACTION [OPTIONS]

The actions that can be defined are:
pick - picks up the cargo
drop - drops the cargo

The options that can be defined are:
cargo_pose     - a pose defined in quaternions of the cargo bag

# Demo example

Launch isaac simulation normally

Spawn cargo:

    roslaunch isaac_gazebo spawn_object.launch spawn:=cargo pose:="11.3 -5.6 5.6 -0.707 0 0 0.707" name:=CTB_05_1070

Pick up cargo (make sure astrobee is undocked) - the pose is the cargo pose inserted above:

    rosrun cargo cargo_tool -id CTB_05_1070 -pick -pose "11.3 -5.6 5.6 -0.707 0 0 0.707"

Drop cargo - the pose is the dock pose:

    rosrun cargo cargo_tool -drop -pose "10.4 -5.6 5.855 -0.707 0 0 0.707"

# ISS cargo berths below the Astrobee dock

The ISS simulation contains two cargo-sized berths at the bottom of the two
Astrobee dock bays. Both berths are fully opaque. The green berth is available
for cargo; the blue berth has a black X and is permanently blocked.

The cargo mesh and each berth have the same dimensions and orientation:
`0.266792 x 0.464894 x 0.275782 m`. On release, cargo within the 0.60 m capture
range snaps to the green berth's front surface like a magnet and is fixed to
the ISS. The two volumes touch face-to-face; they do not overlap. To attach a
held cargo to the green berth, use:

    rosrun cargo cargo_tool -drop \
      -pose "10.0292347 -9.806 4.7411744 0 0 -0.7071068 0.7071068"

The corresponding blue berth command is deliberately rejected with the
`BERTH_FULL` result before Astrobee begins moving:

    rosrun cargo cargo_tool -drop \
      -pose "10.0292347 -10.312 4.7411744 0 0 -0.7071068 0.7071068"

## Return Astrobee to the dock after a cargo drop

The cargo drop sequence finishes outside the dock behavior's normal 1.0 m
starting tolerance. The local dock tool enables return-from-afar by default, so
this command first plans Astrobee back to the approach point and then docks it
in berth 1:

    rosrun dock dock_tool -dock

Use `-noreturn_dock` only when close-only docking is desired. From the cargo
retreat pose, close-only docking is expected to return
`Too far from dock (Code -15)`.
