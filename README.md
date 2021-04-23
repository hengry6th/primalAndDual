# primalAndDual
代码完成了Primal方法和Dual方法对绳子和布料的仿真，并对比了两种方法在不同的mass ratio和 stiffness ratio下不同的表现。
除此之外，还完成两种方法对碰撞和摩擦的处理。
# Dependency
**blender >= 2.9**

# How to start
1. 在当前目录创建一个`output`文件夹，里面存放生成的`.obj`文件。
2. 通过`blender`的`stop motion obj`插件就可以导入`output`文件夹下所有mesh形成的动画。


# Reference
- Kenny Erleben. 2017. Rigid body contact problems using proximal operators. In Proceedings of the ACM SIGGRAPH / Eurographics Symposium on Computer Animation (SCA '17). Association for Computing Machinery, New York, NY, USA, Article 13, 1–12. DOI:https://doi.org/10.1145/3099564.3099575
- M. Macklin, K. Erleben, M. Müller, N. Chentanez, S. Jeschke, and T. Y. Kim. 2020. Primal/dual descent methods for dynamics. In Proceedings of the ACM SIGGRAPH/Eurographics Symposium on Computer Animation (SCA '20). Eurographics Association, Goslar, DEU, Article 9, 1–12. DOI:https://doi.org/10.1111/cgf.14104
