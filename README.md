# ToF-SL-3D-Hybridization-Project-Using-BP-MAP-MRF-

#Projects KinectV1 and KinectV2 access each Kinect individually and is able to save frames. \
  #KinectV1 uses OpenNI2 with a grabber class \
  #KinectV2 uses the NtKinect header file file 

#BP-Test 2.0 is a self-contained project for processing images using the Belief Propogation Algorithm. \
  #Only 1 .cpp file should be used in the solution at a time. \
  #restore.cpp -> 1 input img, applies BP to restore img. \
  #stereo.cpp -> 2 inuput imgs, applies BP to find disparity map. \
  #noise.cpp -> 1 input img, adds noise to img. \
***Note: Does not work with .tif or .jpeg file formats. Code uses its own image class for reading and writing. \
***Valid image formats included in the pnmfile.h file. (Includes .pgm, .pbm, .ppm) \
***Code to convert from openCV Mat object to image class can be found in CombineKinects 

#Combine Kinects uses code from KinectV1 and KinectV2 to grab frames from Kinects. Registers and Crops images based on given  homography matrix (Found using MATLAB for our case). Sends images through BP code to provide an hybridized image. \
  #Main.cpp -> Includes reading the images, registering images, cropping images and converting from a Mat object to Image object. \
  #Grabber.cpp -> From KinectV1 converted into a class. Used to access the SL Kinect camera \
  #Restore.cpp -> Class for BP algorithm. Has 1 public method (restore_ms), takes 2 input images, outputs restored img \
    #Possible Gray level values should be set by the constant VALUES.  \
    #ALPHA biases towards the second input image pixel values, BETA biases towards the first input image pixel values 
