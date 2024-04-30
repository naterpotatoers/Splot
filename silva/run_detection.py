from edge_tpu_silva import process_detection

# Run the object detection process
outs = process_detection(model_path='192_yolov8n_full_integer_quant_edgetpu.tflite', input_path='forest_walk.mp4', imgsz=192, threshold=0.5)

for objs_lst, fps in outs:
    # Access the output parameters as needed
    print(f"Processed frame with {len(objs_lst)} objects. FPS: {fps}")
    print("List of object predictions in frame:")
    print(objs_lst)



# from edge_tpu_silva import process_detection
# import cv2

# # Open the video stream
# cap = cv2.VideoCapture('udp://@<listening_ip>:<port>')

# try:
#     # Run the object detection process
#     outs = process_detection(model_path='192_yolov8n_full_integer_quant_edgetpu.tflite', input_path=cap, imgsz=192, threshold=0.5)

#     for objs_lst, fps in outs:
#         # Access the output parameters as needed
#         print(f"Processed frame with {len(objs_lst)} objects. FPS: {fps}")
#         print("List of object predictions in frame:")
#         print(objs_lst)
# finally:
#     cap.release()
