# 球形位姿图数据

`sphere.g2o`来自《视觉SLAM十四讲》第二版配套仓库的
[slambook2/ch10/sphere.g2o](https://github.com/gaoxiang12/slambook2/blob/master/ch10/sphere.g2o)，本目录保留原始文件。

- 2500条`VERTEX_SE3:QUAT`记录：位姿初值。
- 9799条`EDGE_SE3:QUAT`记录：相对位姿测量及信息矩阵。
- SHA-256：`be8dbad53b43695bfa3246add2f92307c3d7340fc5a5641a6f3e46e3e7d0fc61`。

上游采用MIT许可证，本目录的`LICENSE.slambook2`保留其版权及许可声明。
