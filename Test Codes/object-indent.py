# Import OpenCV
import cv2

# Load class names
classNames = []
classFile = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

# Load model config and weights
configPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/eutech/Desktop/PawStopper-Robot/Object_Detection_Files/frozen_inference_graph.pb"

# Initialize detection model
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Function to detect objects and mark centers
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nms)
    objectInfo = []
    if len(objects) == 0: objects = classNames

    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            className = classNames[classId - 1]
            if className in objects:
                objectInfo.append([box, className])

                if draw:
                    # Draw rectangle
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                    # Draw class name
                    cv2.putText(img, className.upper(), (box[0]+10, box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                    # Draw confidence
                    cv2.putText(img, str(round(confidence * 100, 2)) + '%', (box[0]+200, box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

                    # Compute center of the bounding box
                    center_x = box[0] + box[2] // 2
                    center_y = box[1] + box[3] // 2

                    # Draw small circle at the center
                    cv2.circle(img, (center_x, center_y), radius=5, color=(0, 255, 0), thickness=-1)

    return img, objectInfo

# Main program
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)
    cap.set(4, 480)

    # Optional: print frame size
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video feed size: {frame_width}x{frame_height}")

    while True:
        success, img = cap.read()
        if not success:
            print("Failed to read from camera.")
            break

        # Detect objects and mark centers of their bounding boxes
        result, objectInfo = getObjects(img, 0.45, 0.2, objects=['cup'])

        cv2.imshow("Output", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
