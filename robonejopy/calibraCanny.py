import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Calibration')
cv2.createTrackbar('Canny Thresh1', 'Calibration', 50, 300, nothing)
cv2.createTrackbar('Canny Thresh2', 'Calibration', 150, 300, nothing)
cv2.createTrackbar('Hough Thresh', 'Calibration', 100, 300, nothing)
cv2.createTrackbar('Min Line Length', 'Calibration', 100, 300, nothing)
cv2.createTrackbar('Max Line Gap', 'Calibration', 10, 100, nothing)
cv2.createTrackbar('Brightness', 'Calibration', 50, 100, nothing)
cv2.createTrackbar('Contrast', 'Calibration', 50, 100, nothing)

cap = cv2.VideoCapture(0)  # or 'your_video.mp4' for a video file

while True:
    ret, frame = cap.read()
    if not ret:
        break
    img = cv2.resize(frame, (160, 120))  # Redimensionar para 640x480 (pode ajustar conforme necessário)
    brightness = cv2.getTrackbarPos('Brightness', 'Calibration')
    contrast = cv2.getTrackbarPos('Contrast', 'Calibration')
    canny_thresh1 = cv2.getTrackbarPos('Canny Thresh1', 'Calibration')
    canny_thresh2 = cv2.getTrackbarPos('Canny Thresh2', 'Calibration')
    hough_thresh = cv2.getTrackbarPos('Hough Thresh', 'Calibration')
    min_line_length = cv2.getTrackbarPos('Min Line Length', 'Calibration')
    max_line_gap = cv2.getTrackbarPos('Max Line Gap', 'Calibration')

    # Adjust brightness and contrast
    adjusted_frame = cv2.convertScaleAbs(img, alpha=contrast / 50.0, beta=brightness - 50)
    print("brilho: ",brightness,", contraste: ", contrast)
    gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, hough_thresh, minLineLength=min_line_length, maxLineGap=max_line_gap)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(adjusted_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow('Calibration', adjusted_frame)
    cv2.imshow('Edges', edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
