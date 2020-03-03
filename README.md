# general_task_control

***机器人通用任务控制***

## 1. MotionBase && MotionGeneral 说明
&nbsp;&nbsp;&nbsp;&nbsp;MotionBase为运动控制的抽象类；MotionGeneral为运动控制的通用实现类。  
&nbsp;&nbsp;&nbsp;&nbsp;若默认提供的MotionGeneral的实现不符合实际业务，可参考MotionGeneral，即：继承MotionBase并实现motion函数，并在MotionControl.cpp中的actualMotion实例化为重写之后的对象即可。

## 2. WorkBase && WorkGeneral 说明 
&nbsp;&nbsp;&nbsp;&nbsp;（类似MotionBase && MotionGeneral）WorkBase为运动控制的抽象类；WorkGeneral为运动控制的通用实现类。  
&nbsp;&nbsp;&nbsp;&nbsp;所有的机器人都应当继承WorkBase并根据实际业务实现work函数 。

## 3. general_task_ control自测
```shell
    roslaunch general_task_control inner_test.launch
```
&nbsp;&nbsp;&nbsp;&nbsp;**注意：** send_task_data节点需要task_data.json文件，请先将该文件复制到~/.ros/目录下。

## 4. 启动
```shell
    rosrun general_task_control general_task_control
```