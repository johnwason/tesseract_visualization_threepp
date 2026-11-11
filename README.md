# tesseract_visualization_threepp

A `tesseract::visualization::Visualization` plugin that renders in process with
[threepp](https://github.com/markaren/threepp), in a GLFW window or headless through EGL. There is no
server or browser.

`conversions.h` builds the threepp scene graph directly from a tesseract environment and markers. It has no
dependency on the viewer, but including it needs C++20. The viewer's own header stays C++17.

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

## Using it

```bash
export TESSERACT_VISUALIZATION_PLUGIN_DIRECTORIES=<prefix>/lib
export TESSERACT_VISUALIZATION_PLUGINS=tesseract_visualization_threepp_plugin
```

`VisualizationLoader::get("TesseractThreeppVisualizationPlugin")` then returns the viewer. `get()` with no name also
works, as the plugin exports the Ignition symbol that it asks for. If several listed plugins export it, library search
order decides which loads.

| Environment variable | Plugin config | Default | Meaning |
| --- | --- | --- | --- |
| `TESSERACT_VISUALIZATION_THREEPP_MODE` | `mode` | `auto` | `window`, `headless`, or `auto`: a window if `DISPLAY` is set |
| `TESSERACT_VISUALIZATION_THREEPP_WIDTH` | `width` | 1280 | Pixels |
| `TESSERACT_VISUALIZATION_THREEPP_HEIGHT` | `height` | 720 | Pixels |
| `TESSERACT_VISUALIZATION_THREEPP_LIGHTING` | `lighting` | `hemisphere` | `hemisphere`, or `environment` to add an environment map for PBR materials, like the three.js client's `?lighting=environment` |

A window has orbit controls and plays trajectories in a loop. Headless, use `ThreeppVisualization` directly:
`renderImage()` returns the pixels, `saveImage(path)` writes a file, and `seekTrajectory(time)` picks the frame.

## Examples

| Program | Shows |
| --- | --- |
| `tesseract_visualization_threepp_abb_irb2400_viewer [image.png]` | A robot and an animated trajectory |
| `tesseract_visualization_threepp_pbr_models_viewer <models dir> [image.png]` | glTF sample models using every PBR texture slot, and alpha, emissive and normal map test boards, with environment lighting |

With an image path, an example renders one frame headlessly to it instead of opening a window.

The PBR models example needs models from the Khronos [glTF-Sample-Assets](https://github.com/KhronosGroup/glTF-Sample-Assets), which are not part of this repository. [examples/download_pbr_models.py](tesseract_visualization_threepp/examples/download_pbr_models.py) downloads them at a pinned commit, using only the Python standard library, and writes the scene URDF and an `ATTRIBUTION.md` with each model's license (CC0 or CC-BY). `--include-noncommercial` adds DamagedHelmet, which is partly CC-BY-NC.

```bash
python3 tesseract_visualization_threepp/examples/download_pbr_models.py pbr_models
tesseract_visualization_threepp_pbr_models_viewer pbr_models
```

## Limitations

- One object renders per process at a time, because threepp's OpenGL function pointers are process wide. Any other
  logs an error and stays disconnected until the first is destroyed.
- Headless is Linux only, as it uses EGL. It uses a GPU if there is one and otherwise renders in software.
- macOS cannot open a window, as GLFW needs the main thread there and this viewer renders on its own thread.
- Wayland needs XWayland, as threepp builds GLFW without Wayland support.
- Lines are one pixel wide, and marker labels are not drawn.
- Rendering has only been tested on Linux. CI builds on macOS and Windows and runs the conversion tests there.
