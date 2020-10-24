## `SLAM14`讲第二版项目对于`mac`系统的适配
`mac` 是非常`傲娇`，要求关于图像渲染的操作要在`主线程`实现(Pangolin,OpenCV都涉及图像渲染)，所以原始的`slambook2`项目的代码无法在mac正常运行。

本代码是修改`slambook2/ch13`程序,将前端放在新的线程，将可视化的部分放在主线程,使得能够在`mac`上正常运行(当然`ubuntu`上也能正常运行)。

[原始项目](https://github.com/gaoxiang12/slambook2/tree/master/ch13):https://github.com/gaoxiang12/slambook2/tree/master/ch13

相关库的版本和配置和原始项目相同
### 修改文件
`visual_odometry.cpp`
`visual_odometry.h`
`viewer.h`
`viewer.cpp`
`run_kitti_stereo.cpp1`

### 运行效果
![](https://tva1.sinaimg.cn/large/0081Kckwgy1gk0ha5w5h8j31c00u0e82.jpg)

