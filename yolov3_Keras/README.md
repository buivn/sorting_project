# CVP_Yolov3_Keras
All the codes to perform the computer vision project which is related to Yolov3-Keras

**resizeImage.py:** 
load all iamge files in a folder, resize to the predefined shape and save to one folder. It is run by python 2 and use openCV functions

**imageAugmentation.py:** 
Implement the image augmentation (rotate, flip, shade, or add noise). It run on python2 and Opencv functions\

**makeYolov3Model.py** This function loads the yolov3 weights file (ouput of a Yolov3 whose detection is for 4 classes), integrate with the yolov3 structure in this function then create a yolov3 model to detect objects\

**cv_Yolov3Keras**
This function loads yolov3 model (output of function makeYolov3Model.py) and detects 4 kinds of trained objects in a list of 20 images in folder testImg\

**labelImg** 
This folder contain a program to label the images, the number and name of the classes can be changed in .data file\

**testImg** 
This folder contains 20 images for testing\

**cv_Yolov3Camera.py** 
This program can run Yolov3 with camera as data input. It requires package cv2 - OpenCV to run the program.\
