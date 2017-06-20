# VISGSim


VISGSim is a GPU-based cloth simulator.

![Results](https://github.com/sutongkui/simulator/raw/master/Pic/Fig_1.jpg)

# CONTRIBUTIONS

  - A unified streaming pipeline for time integration and collision handling
  - Unified collision handling: we present a parallel, integrated collision detection and response algorithm - Position projection and inelastic collision response force
  - A new strategy of expanding bounding box(aabb) for the triangle (caused by our collision detection and response strategy)
  - Maximizing Parallelism in the construction of BVHs


### Here is a [Demo](https://youtu.be/e-qQirf1UiY).
  - 12367 vertices
  - intel core i5 and GTX 960
  - nearly 1000 FPS
  
### Operations
* space - start or stop the simulation
* 'W'/'w' - down in the axis '-y'
* 'S'/'s' - up in the axis 'y'
* 'X'/'x' - snapshot
* mouse - press *middle* and drag to scale, press *left* and drag to rotate

# Realated papers
* [parallel bvh construction and implementation](https://devblogs.nvidia.com/parallelforall/thinking-parallel-part-ii-tree-traversal-gpu/)
*  Contact-Aware Matrix Assembly with Uniﬁed Collision Handling for GPU-based Cloth Simulation
# contact
If you need the corresponding testing model, such as body model, garments model, just e-mail - tongkuisu@smail.nju.edu.cn



