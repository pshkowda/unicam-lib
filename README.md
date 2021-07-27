# unicam-lib

 Provides a common interface to grab frames from xtion2 (openNI), realsense2 (librealsense2) devices and Kinect (using freenect2)

`UnicamDeviceProvider`: allows the user to connect different cameras and access them through a common interface,
`UnicamCamera` object provides access to a single camera.

```
    UnicamDeviceProvider *xtion = new XtionProvider();
    UnicamDeviceProvider *freenect = new FreenectProvider();
    UnicamDeviceProvider *realsense = new RealsenseProvider();
    
    freenect->initializeCameras();
    xtion->initializeCameras();
    realsense->initializeCameras;
    
    UnicamCamera *camera = realsense->getCameraByTag("8176******");
    while (true) {
        realsense->spinOnce();  //updates the frameset to current
        cv::Mat matrix = camera->getDepthFrame();
      }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      return (0);
```
