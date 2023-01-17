# TiagoRobotSkillLearning
![Alt Text](https://github.com/btknzn/TiagoRobotSkillLearning/blob/main/TIAGo-Research.jpg)


pick_and_place: This node is developed GUI used blue object detection based on blue point cloud, approaching to object, openning Tiago Robot arm, Picking Object and
placing object for Tiago Robots. 

![Alt Text](https://github.com/btknzn/TiagoRobotSkillLearning/blob/main/tiagoactionGUI.png)

Example Video : https://youtu.be/nKSOe_63HZI


Another Example demo: https://www.youtube.com/watch?v=zSLZpDB0UUA



You can use record.py file located in pick_and_place/scripts/ for recording current trajectories, images and also diparities for motion. System is based on multi thread qt, so press start recording for starting recording, press stop for stop recording and for saving all data collected between start and stop press save button.


![Alt Text](https://github.com/btknzn/TiagoRobotSkillLearning/blob/main/record.png)


Due to less computational power of my hardware( NVidia GTX 1650), learning part could not completed based on above mentioned methods - or could not collected big dataset. Instead of one easy learning task is recorded based on this tool: http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place
(main problem is related above method, gtx1650 does not have sufficient memory to have very big models to learn complex motions and it works slowly for my pc) 

Colected dataset located here: https://drive.google.com/drive/folders/1OW1NMhZRi9S7oYW4D1I_T8BLPN0KyScm?usp=sharing for tiago implemented motion here.



The Neural Network(for Imitation learning) trained has the below structure.


![Alt Text](https://github.com/btknzn/TiagoRobotSkillLearning/blob/main/PredictiveLearningArct.png)

Imitation learning based object touching example videos:

1. https://youtu.be/gRxLfBI0K94

For different object location:

2. https://youtu.be/HcIxalXrmVY

play: this node is for using the model trained. Place your models there and use play.py code for tiago.(change normalization parameters in play.py)

Hiearcial order of predictive learning was like that train encoder-decoder for RBG Images and Disparity
Afterwards train encoder lstm mlp for grasping.
