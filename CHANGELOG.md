# Changelog

[3.1.0] - 2024-08-19
- added step height for 3D movement
- added min velocity for grounded event on 2D/3D movement

[3.0.0] - 2024-07-05
- fixed missing headers on new properties

[3.0.0] - 2024-06-21
- switched all fields to properties, many with public sets
- fixed issues with 2D hybrid (2D sprite + 3d movement) characters not turning correctly

[2.4.5] - 2024-05-29
- added CanTurn alongside CanMove

[2.4.4] - 2024-05-04
- added toggle to fix sprite shaders no longer flipping

[2.4.1] - 2023-11-16
- fixed rotation interpolation issues

[2.4.0] - 2023-10-25
- added experimental local avoidance for 3D character movement

[2.3.0] - 2023-8-24
- added SetLookPosition to movement

[2.2.1] - 2023-8-24
- added height/radius properties to movement
- added more auto-configuration of components in OnValidate

[2.1.1] - 2023-7-07
- reduced moving platform jitter

[2.1.0] - 2023-3-01
- added cinemachine sensitivity component

[2.0.8] - 2023-3-01
- removed runtime set of rigidbody2D configuration
- fixed manual configuration of rigidbodies in example scenes

[2.0.7] - 2023-02-25
- added stopping distance
- removed runtime set of rigidbody configuration
- switched navmeshagent sync to nextPosition (warp was resetting the path in 2021.3.19f1)
- added rigidbody mass to acceleration calculation

[1.5.0] - 2022-11-15
- Added Footstep/Grounded UnityEvents

[1.4.0] - 2022-11-11
- Added MovingPlatform component
- Adding example moving platforms to sample scenes

[1.3.0] - 2002-10-26
- Simplified movement scripts, removed unnecessary Update/LateUpdate calls
- Removed time requirement from coyote jump check
- made all fields/methods protected to allow for specific movement implementations

[1.2.1] - 2022-10-26
- Fixed null ref in 2D platformer sample

[1.2.0] - 2022-10-26
- All sample scenes are in

[1.1.1] - 2022-10-24
- Added Common Assets sample for shared assets used in other samples.

[1.1.0] - 2022-10-24
- Added 1st person demo sample.

[1.0.0] - 2022-10-23
## Added
- This is the first release of Character Movement, as a package.