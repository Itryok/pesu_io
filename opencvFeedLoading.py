import cv2
print(cv2.__version__)
cap = cv2.VideoCapture(0)

while True:
    success,img = cap.read()
    cv2.imshow('video',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
print('success')
cv2.destroyAllWindows()
video.release()
