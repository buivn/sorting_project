# this file run by python 2

import numpy as np
import cv2
import random
import os
import glob

if __name__ == '__main__':

    inputdir = '/home/bui/Desktop/images07012019/'
    outputdir = '/home/bui/Desktop/resize07012019/'
    #filelist = os.listdir(inputdir)
    data_path = os.path.join(inputdir,'*g')
    filelists = glob.glob(data_path)
     
    width = 640
    height = 480
    dim1 = (width, height)
    dim2 = (height, width)
    i = 0    

    for filename in filelists:
        #img = cv2.imread('/home/bui/Desktop/images07012019/IMG20190701192106.jpg', cv2.IMREAD_UNCHANGED)
        img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)

        name = filename[-21:] 
        #print(name_int)  
        print('Image ', i)   
        print('Original Dimensions : ',img.shape)   
        if (img.shape[0] > img.shape[1]):
            # resize image
            resized = cv2.resize(img, dim2, interpolation = cv2.INTER_AREA)
        else:
            resized = cv2.resize(img, dim1, interpolation = cv2.INTER_AREA)
             
        print('Resized Dimensions : ',resized.shape)
        cv2.imwrite(outputdir+name, resized)
        i += 1

    # cv2.imshow("Resized image", resized)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows() 