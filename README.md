# URDF Editor

This is a WYSIWYG editor that allows you to create, modify and
visualize robots defined with [URDF](http://wiki.ros.org/urdf). The main goal of
this project is to create an editor that does not depend on the ROS software
stack and is platform agnostic, you only need a C++17 compiler.

![Editor screenshot](./resources/screenshot.png)

# Features

- Create new robots or load existing URDF files
- Interactive robot tree with drag-and-drop to reparent joints
- 3D gizmo to visually position links and joints
- Edit joint properties: type, axis, dynamics, limits
- Edit link properties: inertial (mass, inertia tensor), visual, collision
- Multiple collision geometries per link
- Geometry types: box, cylinder, sphere, mesh (`.dae`, `.stl`)
- Full undo/redo for all operations
- Save to URDF

# Install

All C++ dependencies are automatically downloaded and compiled via CMake
FetchContent. You only need a C++17 compiler and CMake 3.10+.

## Linux

Install the required system libraries first:

```bash
sudo apt install pkg-config libgtk-3-dev libxi-dev libxcursor-dev libxinerama-dev libxrandr-dev
```

Then build:

```bash
git clone https://github.com/noctrog/urdf-editor && cd urdf-editor
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/urdf-editor
```

## macOS

No extra system libraries are needed. Just build with CMake:

```bash
git clone https://github.com/noctrog/urdf-editor && cd urdf-editor
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(sysctl -n hw.ncpu)
./build/urdf-editor
```

# Usage

## Camera Controls

| Input                        | Action          |
|------------------------------|-----------------|
| Middle mouse button + drag   | Orbit           |
| Shift + middle mouse + drag  | Pan             |
| Scroll wheel                 | Zoom            |

## Keyboard Shortcuts

| Shortcut             | Action              |
|----------------------|----------------------|
| Ctrl/Cmd + O         | Open URDF file       |
| Ctrl/Cmd + S         | Save URDF file       |
| Ctrl/Cmd + N         | New robot            |
| Ctrl/Cmd + J         | Create joint (when a link is selected) |
| Ctrl/Cmd + Z         | Undo                 |
| Ctrl/Cmd + Y         | Redo                 |
| Ctrl/Cmd + Shift + Z | Redo                 |

# Mesh Path Resolution

URDF files often reference mesh geometries (`.dae`, `.stl`) using different path
styles. The editor resolves them as follows:

- **`package://` paths** (common in ROS): The editor uses the package name in
  the URI. It first checks whether the URDF is inside that package, then searches
  the entries in `ROS_PACKAGE_PATH`. Same-package paths therefore work without
  ROS installed; references to another package require a sourced ROS environment
  that exposes the target package.

- **Relative paths**: Resolved relative to the directory containing the URDF
  file.

- **Absolute paths**: Used as-is.

If you're working outside of a ROS workspace, the simplest approach is to place
your mesh files relative to the URDF file and use relative paths in the
`<mesh filename="..."/>` attribute. If your URDF uses `package://` paths for its
own package, make sure its matching `package.xml` is in a parent directory. For
cross-package paths, source the ROS workspace before starting the editor.

You can also change the mesh file at runtime by clicking the `...` button next
to the filename in the Geometry section and selecting a new `.dae` or `.stl`
file.

# Acknowledgements

- [raylib](https://github.com/raysan5/raylib) - A simple and easy-to-use library to enjoy videogames programming
- [raygizmo](https://github.com/alexeykarnachev/raygizmo) - A simple 3D transformation gizmo for raylib

# Contributing

Contributions are always welcome! Feel free to open an issue or create a merge
request :)
