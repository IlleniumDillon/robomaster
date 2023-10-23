# robomaster
## 项目功能

识别出双目相机中红圈的中心点位置和其法向量

## 使用方法

主要函数为`IntelligentUAVChampionship-vision.cpp`和`IntelligentUAVChampionship-vision.h`

将这两个函数放在项目之中，并`include`其头文件，调用`imageMain()`主函数即可，输入双目相机的两张图片，在画面同时出现红圈时即可输出该红圈的中心点位置及其法向量。