# CAD Replacement and Interactive Scene Generation

We provide a ScanNet scene file `scannet_test` with ground-truth part annotation for scene 157, which is under the folder `input/`. To generate the interactive scene, please follow the instructions below.

## CAD Replacement and Contact Graph Generation

First, to replace the parts with CAD models and generate the contact graph, run the following command. Please make sure you activate the conda env beforehand.

The input format of our program is as following:
```test
<input-folder>/
    <object-folder-1>/
        ...
    <object-folder-2>/
    ...
    <scene-point-cloud.ply>
```
More details please refer to our [example input](../input/scannet_test).

```shell
conda activate part-level-scene-recon
cd <your-ros-ws>/src/Part-Mesh-Reconstruction
python part2cad/app/cvt_scene.py --src input/scannet_test --loader gt
```

The output will be saved in the `input/` folder under `Part-Mesh-Reconstruction/scene_builder/`.

## Interactive Scene Generation

We provide a scene builder tool to generate the interactive scene from the constructed contact graph and the replaced parts. You can generate the xacro scene by:

```shell
conda activate part-level-scene-recon
cd <your-ros-ws>
roslaunch scene_builder generate_xacro_scene.launch scene_name:=scannet_test
```

We also provide some [launch files](../scene_builder/launch/) to visualize the constructed virtual interactive scene. For example, use

```shell
roslaunch scene_builder view_scene.launch scene:=scannet_test
```