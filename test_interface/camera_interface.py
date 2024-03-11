import cv2

# config video stream
cap = cv2.VideoCapture(0, cv2.CAP_V4L)

while True:
	ret, frames = cap.read()
	if ret:
		frames = frames[:, :, 0]
		cv2.imshow('farame', frames)
		cv2.waitKey(1)
	else:
		cap = cv2.VideoCapture(0)
