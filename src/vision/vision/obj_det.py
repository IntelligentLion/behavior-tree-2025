

import cv2
import numpy as numpy
import pyzed.sl as sl

zed=sl.Camera()

init_params=sl.InitParameters()
init_params.camera_resolution=sl.RESOLUTION.HD720
init_params.coordinate_units=sl.UNIT.METER
init_params.depth_mode=sl.DEPTH_MODE.PERFORMANCE
init_params.sdk_verbose = 0

err=zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("failed to open camera")
    exit(1)

obj_param=sl.ObjectDetectionParameters()
obj_param.enable_tracking=True
obj_param.enable_segmentation=True
obj_param.detection_model = "/home/yirehban/Downloads/model-1-yolov8n.onnx"

if obj_param.enable_tracking:
    positional_tracking_param=sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(positional_tracking_param)

err=zed.enable_object_detection(obj_param)

objects=sl.Objects()
obj_runtime_param=sl.ObjectDetectionRuntimeParameters()
obj_runtime_param.detection_confidence_threshold=30
obj_runtime_param.detection_confidence_threshold=30

font = cv2.FONT_HERSHEY_SIMPLEX

# org
org = (50, 50)

# fontScale
fontScale = 1
 
# Blue color in BGR
color = (255, 0, 0)

# Line thickness of 2 px
thickness = 2

cv2.namedWindow("ZED", cv2.WINDOW_NORMAL)

while True:

    if err!=sl.ERROR_CODE.SUCCESS:
        print(f"camera failed")
    if zed.grab()==sl.ERROR_CODE.SUCCESS:
        zed.retrieve_objects(objects, obj_runtime_param)
        img=sl.Mat()
        zed.retrieve_image(img, sl.VIEW.RIGHT)
        img_cv=img.get_data()
        
        if objects.is_new:
            obj_arr=objects.object_list
            print(str(len(obj_arr)))

            for obj in obj_arr:
                top_left=obj.bounding_box_2d[0]
                bottom_right=obj.bounding_box_2d[2]

                cv2.rectangle(img_cv, (int(top_left[0]), int(top_left[1])), (int(bottom_right[0]), int(bottom_right[1])),(0,255,0), 2)
                label=f"{obj.label} ({int(obj.confidence)}%)"
                cv2.putText(img_cv, label, (int(top_left[0]), int(top_left[1]-10)), font, fontScale, color, thickness, cv2.LINE_AA)
        
        cv2.imshow("Object detection: ", img_cv)

        if cv2.waitKey(1) & 0xFF==ord('q'):
            break

zed.disable_object_detection()
zed.close()
cv2.destroyAllWindows()