# rosbag_utils
Bunch of utility scripts I use to process rosbags. Some are borrowed & modified with credits

## Resizing images and camera_info msgs:
Use `resize_image_and_cam_info.py`, which figures out the original width and height from the camera_info msgs, and resizes the images and changes camera intrinsics(`cam_info.K`) and projection (`cam_info.P`) projection matrices accordingly:   
```
python resize_image_and_cam_info.py 
    --in_bag_file in_bag.bag 
    --out_bag_file out_bag.bag 
    --image_topic "/camera/image_raw" 
    --cam_info_topic "/camera/camera_info" 
    --desired_width 640 
    --desired_height 480
```
