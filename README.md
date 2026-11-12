# tesseract_visualization_threepp

Rendering of [Tesseract](https://github.com/tesseract-robotics/tesseract) environments in process with
[threepp](https://github.com/markaren/threepp), with no server or browser. There are two packages:

| Package | What it is |
| --- | --- |
| `tesseract_threepp` | Builds a threepp scene from a Tesseract environment, its markers and trajectories. No window, camera or OpenGL, so viewers and simulated sensors can share it. Its headers expose threepp, so using it needs C++20 |
| `tesseract_visualization_threepp` | A `tesseract::visualization::Visualization` plugin built on it, which renders in a GLFW window or headless through EGL. Its header stays C++17 |

## Building

threepp and the other source dependencies are pinned in `dependencies.repos`. This builds everything in a container:

```bash
vcs import src < dependencies.repos
tools/build_image.sh
tools/build.sh
```

Outside `tools/build.sh`, pass `-DBUILD_SHARED_LIBS=ON -DTHREEPP_BUILD_EXAMPLES=OFF -DTHREEPP_BUILD_TESTS=OFF
-DTHREEPP_BUILD_EDITOR=OFF -DTHREEPP_WITH_AUDIO=OFF` when building threepp. `tools/test_threepp_window.sh` runs the
tests on a virtual display, which adds the GLFW window test.

## Using the plugin

```bash
export TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
export TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threepp_factories
```

`VisualizationLoader::get("TesseractThreeppVisualizationPlugin")` then returns the viewer. `get()` with no name also
works, as the plugin exports the Ignition symbol that it asks for. If several listed plugins export it, library search
order decides which loads.

| Environment variable | Plugin config | Default | Meaning |
| --- | --- | --- | --- |
| `TESSERACT_VISUALIZATION_THREEPP_MODE` | `mode` | `auto` | `window`, `headless`, or `auto`: a window if `DISPLAY` is set |
| `TESSERACT_VISUALIZATION_THREEPP_WIDTH` | `width` | 1280 | Pixels |
| `TESSERACT_VISUALIZATION_THREEPP_HEIGHT` | `height` | 720 | Pixels |

A window has orbit controls and plays trajectories in a loop. Headless, use `ThreeppVisualization` directly:
`renderImage()` returns the pixels, `saveImage(path)` writes a file, and `seekTrajectory(time)` picks the frame.

## Using the scene directly

`tesseract::threepp::TesseractScene` holds the environment, joint state, markers and trajectory playback as one threepp
scene. Render `scene()` with any threepp renderer and camera. `conversions.h` has the pieces it is built from.

## Limitations

- One object renders per process at a time, because threepp's OpenGL function pointers are process wide. Any other
  logs an error and stays disconnected until the first is destroyed.
- Headless is Linux only, as it uses EGL. It uses a GPU if there is one and otherwise renders in software.
- macOS cannot open a window, as GLFW needs the main thread there and this viewer renders on its own thread.
- Wayland needs XWayland, as threepp builds GLFW without Wayland support.
- Lines are one pixel wide, and marker labels are not drawn.
- Rendering has only been tested on Linux. CI builds on macOS and Windows and runs the `tesseract_threepp` tests there.
