# Part-Interactive-Scene

![ros-version](https://img.shields.io/badge/ubuntu%2020.04+ROS%20noetic-passing-brightgreen) ![python-version](https://img.shields.io/badge/Python-3.7%2B-blue)

![pipeline](assets/pipeline.png)

This repository implements a pipeline that reconstructs interactive indoor scenes from point clouds. The pipeline replaces parts of objects with primitive shapes and represents the reconstructed scene as a contact graph. The reconstructed scene can be converted to a URDF which reflects objects’ and the scene’s kinematics and can be imported into various simulators to support robot interactions.

## Installation and Usages

- [Installation on Ubuntu](assets/INSTALL.md)

- [CAD Replacement and Interactive Scene Generation](assets/USAGES.md)

## [Project Page](https://zeyuzhang.com/papers/2023-iros-part-scene/)

## Citing

- Zeyu Zhang\*, Lexing Zhang\*, Zaijin Wang, Ziyuan Jiao, Muzhi Han, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Part-level Scene Reconstruction Affords Robot Interaction**, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2023 [[Paper](https://yzhu.io/publication/scenereconstruction2023iros/paper.pdf)] [[Arxiv](https://arxiv.org/abs/2307.16420)]

```bibtex
@inproceedings{zhang2023part,
  title={Part-level Scene Reconstruction Affords Robot Interaction},
  author={Zhang, Zeyu and Zhang, Lexing and Wang, Zaijin and Jiao, Ziyuan and Han, Muzhi and Zhu, Yixin and Zhu, Song-Chun and Liu, Hangxin},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2023}
}
```

## Related Publications

- Muzhi Han\*, Zeyu Zhang\*, Ziyuan Jiao, Xu Xie, Yixin Zhu, Song-Chun Zhu, and Hangxin Liu. **Reconstructing Interactive 3D Scenes by Panoptic Mapping and CAD Model Alignments**, IEEE International Conference on Robotics and Automation (ICRA), 2021 [[Paper](https://yzhu.io/publication/scenereconstruction2021icra/paper.pdf)] [[Arxiv](https://arxiv.org/abs/2103.16095)]