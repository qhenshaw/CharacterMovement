# Character Movement
[![Unity 2021.1+](https://img.shields.io/badge/unity-2020.1%2B-blue.svg)](https://unity3d.com/get-unity/download)
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/qhenshaw/CharacterMovement/blob/main/LICENSE.md)

Character Movement is a set of `Rigidbody` based movement components. It supports:
- 2D and 3D movement
- all physics (characters can be launched by setting velocity or adding force)
- moving floors/platforms (characters match speed with velocity of rigidbody floor surfaces)
- navigation using `NavMeshAgent` to find paths
- max slope angle for standing/moving
- "coyote time" allows for junping just after leaving platforms

## System Requirements
Unity 2021.1+. Will likely work on earlier versions but this is the version I tested with.

## Installation
Use the Package Manager and use Add package from git URL, using the following: 
```
https://github.com/qhenshaw/CharacterMovement.git
```

## Usage
Install the 3rd Person Demo sample for an example of using the CharacterMovement3D component to drive player movement using the `InputSystem` and `Cinemachine`.

More sample scenes are on the way for the following movement types:
- 2D platformer
- 2D topdown
- 2.5D platformer
- 3D isometric
- First person
