import cv2
import mediapipe as mp
import time

class FaceDetector():
    def __init__(self, minDetectionCon=0.5):

        self.minDetectionCon = minDetectionCon

        self.mpFaceDetection = mp.solutions.face_detection
        self.mpDraw = mp.solutions.drawing_utils
        self.faceDetection = self.mpFaceDetection.FaceDetection(self.minDetectionCon)

    def findFaces(self, img, draw=True):

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.faceDetection.process(imgRGB)
        # print(self.results)
        bboxs = []
        if self.results.detections:
            for id, detection in enumerate(self.results.detections):
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, ic = img.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
                bboxs.append([id, bbox, detection.score])
                if draw:
                    img = self.fancyDraw(img,bbox)

                    cv2.putText(img, f'{int(detection.score[0] * 100)}%',
                            (bbox[0], bbox[1] - 20), cv2.FONT_HERSHEY_PLAIN,
                            2, (255, 0, 255), 2)
        return img, bboxs

    def findCenter(self, bbox):
        x, y, w, h = bbox
        center_x = x + w // 2
        center_y = y + h // 2
        return (center_x, center_y)
    
    def returnCenter(self, img):
        img, bboxes = self.findFaces(img, draw=False)
        max_confidence = 0
        first_box = None
        for box in bboxes:
            if(box[2][0] > max_confidence):
                max_confidence = box[2][0]
                first_box = box
        if first_box == None:
            return
        coordinates = first_box[1]
        center = self.findCenter(coordinates)
        return center

    def getImageDimensions(self, img):
        height, width, channels = img.shape
        return width, height

    def returnRelativePosition(self, center, img):
        width, height = self.getImageDimensions(img)
        if center == None:
            center_x, center_y = 0, 0
        else:
            center_x, center_y = center
    
        # Calculate relative position (0.0 to 1.0)
        rel_x = center_x / width
        rel_y = center_y / height
        return (rel_x, rel_y)
        
    def returnSignal(self, relativeCenter):
        rel_x, rel_y = relativeCenter
        return (rel_x - 0.5, -rel_y + 0.5)
            
    
    def fancyDraw(self, img, bbox, l=30, t=5, rt= 1):
        x, y, w, h = bbox
        x1, y1 = x + w, y + h

        cv2.rectangle(img, bbox, (255, 0, 255), rt)
        # Top Left  x,y
        cv2.line(img, (x, y), (x + l, y), (255, 0, 255), t)
        cv2.line(img, (x, y), (x, y+l), (255, 0, 255), t)
        # Top Right  x1,y
        cv2.line(img, (x1, y), (x1 - l, y), (255, 0, 255), t)
        cv2.line(img, (x1, y), (x1, y+l), (255, 0, 255), t)
        # Bottom Left  x,y1
        cv2.line(img, (x, y1), (x + l, y1), (255, 0, 255), t)
        cv2.line(img, (x, y1), (x, y1 - l), (255, 0, 255), t)
        # Bottom Right  x1,y1
        cv2.line(img, (x1, y1), (x1 - l, y1), (255, 0, 255), t)
        cv2.line(img, (x1, y1), (x1, y1 - l), (255, 0, 255), t)
        #Image Center 
        cv2.circle(img, (int((x+x1)/2), int((y+y1)/2)), 5, (255,0,255), 1)
        return img


def main():
    cap = cv2.VideoCapture(0)
    pTime = 0
    detector = FaceDetector()
    #headSerialController = HeadSerialController()
    
    while True:
        success, img = cap.read()
        img, bboxs = detector.findFaces(img)
        #print(bboxs)
        center = detector.returnCenter(img)
        #print(coordinates)
        relative = detector.returnRelativePosition(center, img)
        #print(relative)
        returnSignal = detector.returnSignal(relative)
        print(returnSignal)

        #headSerialController.send_command(returnSignal)
        
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()