# VISGSim


VISGSim is a GPU-based real time cloth simulator.

![Results](https://github.com/sutongkui/simulator/raw/master/Pic/Fig_1.jpg)

# Contributions

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
* press *left* and drag to rotate
* press *middle* and drag to scale
* press *right* and drag to drag mass

# Realated papers

* [GPU-based Real-time Cloth Simulation for Virtual Try-on](https://diglib.eg.org/handle/10.2312/pg20181288)
* [Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d Trees](https://research.nvidia.com/publication/maximizing-parallelism-construction-bvhs-octrees-and-k-d-trees)
*  [Contact-Aware Matrix Assembly with Uniﬁed Collision Handling for GPU-based Cloth Simulation](http://gamma.cs.unc.edu/CAMA/)

# Compile
Use cmake to compile it and check **CUDA_SEPARATE_COMPILATION**.

following is a simple build script for windows

```powershell
git clone --recursive http://github.com/sutongkui/simulator
# notice: choose your own generator
cmake -G"Visual Studio 15 2017 Win64" -H".\simulator" -B".\simulator\build" -DCMAKE_INSTALL_PREFIX=install
cmake --build ".\simulator\build" --config "RelWithDebInfo" --target INSTALL
# generated files should be in install dir
dir install/bin
```

or you can use **cmake-gui** to do the job in an interactive way.


