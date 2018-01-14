# Quad3DR

*Quad3DR* enables planning of Quadrotor trajectories for dense 3D reconstruction of buildings and other structures.

*Quad3DR* computes a volumetric map of the environment from an initial flight
(using a modified version of [Octomap](https://github.com/OctoMap)). The volumetric map is used to **ensure that the
planned trajectories are only in free space** and not in occupied or unknown space.
Next a viewpoint graph is built by performing a **raycast on the GPU** to measure visible voxels and a viewpoint score is computed.
The viewpoint score is computed based on distance and incidence angle of viewpoint and voxel
(the angle is computed by using a rendered Poisson-reconstructed mesh of the initial flight).
Connections in the viewpoint graph are found using **RRT***. The resulting viewpoint score is **submodular** and we compute
a viewpoint path by using an adaptation of the recursive method in
(Chekuri, Chandra, and Martin Pal. "A recursive greedy algorithm for walks in directed graphs." Foundations of Computer Science, 2005. FOCS 2005. 46th Annual IEEE Symposium on. IEEE, 2005.).

The planned viewpoint paths were evaluated on synthetic scenes using Unreal Engine and on real scenes using a DJI Matrice 100.

## Screenshot
![Screenshot](https://raw.githubusercontent.com/bennihepp/Quad3DR/master/Screenshot.png?token=AA957YRPZYt7btvps5kmQbWGylK5uILMks5aZIikwA%3D%3D)

## Some results from real a scene
![Result](https://raw.githubusercontent.com/bennihepp/Quad3DR/master/RealResults.png?token=AA957eHcVO89NKMB-wPqV9EEQ3DbSj_Fks5aZIihwA%3D%3D)
