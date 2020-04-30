# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import kalmanFilter_filterpy
import bluetooth_manager
import arm_solver

import qrcode

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])

cap = None

if not args.get("video", False):
    cap = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
else:
    cap = cv2.VideoCapture(args["video"])
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the autofocus off
# allow the camera or video file to warm up
time.sleep(2.0)

# establish blob detector
params = cv2.SimpleBlobDetector_Params()
params2 = cv2.SimpleBlobDetector_Params()
params.filterByColor = 1
params.blobColor = 255
params2.filterByColor = 1
params2.blobColor = 255

params.filterByCircularity = 1
params.minCircularity = 0.1
params.maxCircularity = 1.0

params.filterByInertia = 1
params.minInertiaRatio = 0.1
params.maxInertiaRatio = 1.0

params.filterByConvexity = 1
params.minConvexity = 0.1
params.maxConvexity = 1.0

params.filterByArea = 1
params2.filterByArea = 1
params.minArea = 125
params2.minArea = 200

# initialize blob detectors
detector = cv2.SimpleBlobDetector_create(params)
calibrator = cv2.SimpleBlobDetector_create(params2)

# cyan color mask parameqters for hsv
low_red = np.array([0, 0, 50])
high_red = np.array([20, 255, 255])

low_cyan = np.array([89, 50, 50])
high_cyan = np.array([99, 255, 255])

low_green = np.array([31, 30, 30])
high_green = np.array([60, 255, 255])

# grayscale threshold for threshold layer
thresh = 0


# wrapper function for keypoint.size
def get_size(keypoint):
    return keypoint.size


# Connect to raspberry pi
client_socket = bluetooth_manager.conenct_pi_bluetooth()
print("start rasp pi within 10 seconds...")
time.sleep(10)
print("Beginning Best-Friend Bot")

# Callibrate Camera
cali_counter = 0
pix_per_meter = 0
while (1):
    frame = cap.read()
    frame = cv2.flip(frame, 1)
    # crop weird top and bottom frame made from camera
    frame = frame[275:400, 200:450, :]

    # convert to hsv space, use blob detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, low_green, high_green)
    green_mask = cv2.erode(green_mask, None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2)
    green = cv2.bitwise_and(hsv, hsv, mask=green_mask)
    gray = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (5, 5), 0)
    img_binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
    # detect blobs
    # img_binary[250:, :] = 0
    keypoints = calibrator.detect(img_binary)

    img_with_keypoints = cv2.drawKeypoints(img_binary, keypoints, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # calculate running average of pixels between two calibration blobs
    if len(keypoints) == 2:
        (x1, y1) = keypoints[0].pt
        (x2, y2) = keypoints[1].pt
        pix_per_meter = pix_per_meter + int(abs(x1 - x2))
        cali_counter += 1
        if cali_counter >= 5:
            # calculate average pixels per meter, blobs are .1 meters apart so multiply by 10
            pix_per_meter = int(pix_per_meter / 5 * 10)
            print("Calibrated! pix_per_meter = ", str(pix_per_meter))
            print("Place calibration block at robot base and press any key to continue")
            cv2.waitKey(0)
            break

    cv2.imshow('frame', img_with_keypoints)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Callibrate robot arm postion
cali_counter = 0
base_x = 0
base_y = 0
while (1):
    frame = cap.read()
    frame = cv2.flip(frame, 1)
    # crop weird top and bottom frame made from camera
    frame = frame[120:410, 100:600, :]

    # convert to hsv space, use blob detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, low_green, high_green)
    green_mask = cv2.erode(green_mask, None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2)
    green = cv2.bitwise_and(hsv, hsv, mask=green_mask)
    gray = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (5, 5), 0)
    img_binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
    # detect blobs
    keypoints = calibrator.detect(img_binary)

    img_with_keypoints = cv2.drawKeypoints(img_binary, keypoints, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # calculate running average of robot base position
    if len(keypoints) == 1:
        (x1, y1) = keypoints[0].pt
        base_x = base_x + x1
        base_y = base_y + y1
        cali_counter += 1
        if cali_counter >= 10:
            # calculate average base position
            base_x = (base_x / 10 / pix_per_meter)
            base_y = (290 - (base_y / 10)) / pix_per_meter
            print("Calibrated! base position = ", str(base_x), "meters, ", str(base_y), "meters")
            print("Remove Calibration block and press any key to continue")
            cv2.waitKey(0)
            break

    cv2.imshow('frame', img_with_keypoints)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# establish Kalman filter data structures
z = np.array([[0.0], [0.0]])

# obtained through average time for each frame WITH drawing keypoint circles
dt = 0.00864  # s
g = 9.81  # m/s^2
# one filter tracks the entire throw, the other one is used to predict the final interception point
f1 = kalmanFilter_filterpy.init_filter(dt)
f2 = kalmanFilter_filterpy.init_filter(dt)


# convert from predicted states to pixels
def get_coord(x_state, y_state):
    x_coord = int(x_state * pix_per_meter)
    y_coord = int(290 - y_state * pix_per_meter)
    return x_coord, y_coord


# move the ev3 arm
def move_arm(x, y, clamp):
    print(x, y)
    soln, a1, a2 = arm_solver.get_angle(x, y)
    # check if its possible to get to this angle
    if soln == False:
        return

    print("sending!")
    print(a1, a2)
    # rasp pi will wait for a line (newline) before sending to ev3
    # send start bit a, angle1, b, angle2, y or n, \n
    client_socket.send(b'a')
    client_socket.send(str(a1).encode('utf-8'))
    client_socket.send(b'b')
    client_socket.send(str(a2).encode('utf-8'))
    if clamp == True:
        client_socket.send(b'y')
    else:
        client_socket.send(b'n')
    client_socket.send(b'\n')


# get predicted place of interception
def get_interception(img_binary):
    # copy state of base kalman filter (f1) into predictor filter (f2)
    x_old = f1.x
    x_max = 500 / pix_per_meter
    y_max = 290 / pix_per_meter
    max_range = arm_solver.r_max
    curr_pos = f1.x
    predict_num = 0

    # continuously precict steps until an interception is found (or until prediction leaves frame)
    while (1):
        # solution misses interception zone, return failed prediction
        if curr_pos[0] > x_max or curr_pos[0] < 0 or curr_pos[1] > y_max or curr_pos[1] < 0:
            # print("Out of range")
            return img_binary, x_old
        # calculate euclidean between robot base and current point prediction
        curr_r = np.sqrt(np.power((base_x - curr_pos[0]), 2) + np.power((base_y - curr_pos[1]), 2))

        # check to see if point is in interception area, return point of interception
        if curr_r < max_range and curr_pos[0] < base_x and curr_pos[1] > base_y:
            #print('converged!')
            #print(curr_pos)
            # update predict trail
            x, y = get_coord(curr_pos[0], curr_pos[1])
            img_binary = cv2.circle(img_binary, (x, y), 20, (255, 0, 0))
            cv2.imshow('frame', img_binary)
            # if prediction converges within 5 steps, begin closing the grabber
            if predict_num < 5:
                move_arm(base_x - curr_pos[0], curr_pos[1] - base_y, True)
            else:
                move_arm(base_x - curr_pos[0], curr_pos[1] - base_y, False)

            return img_binary, x_old

        # predict next position, continue until interception is found or prediction leaves frame
        f1.predict(u=g)
        curr_pos = f1.x
        x, y = get_coord(curr_pos[0], curr_pos[1])
        img_binary = cv2.circle(img_binary, (x, y), 5, (255, 0, 0))
        predict_num += 1


# time each frame
start_time = time.time()
running_average = 0
time_counter = 0
counter = 0
empty_frames = 0
last_pt = None
initialized = False
while (1):
    frame = cap.read()
    frame = cv2.flip(frame, 1)
    # crop weird top and bottom frame made from camera
    frame = frame[120:410, 100:600, :]

    # invert bgr image, now we want to find the cyan ball
    frame = ~frame
    # convert to hsv space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cyan_mask = cv2.inRange(hsv, low_cyan, high_cyan)
    cyan_mask = cv2.erode(cyan_mask, None, iterations=2)
    cyan_mask = cv2.dilate(cyan_mask, None, iterations=2)
    cyan = cv2.bitwise_and(hsv, hsv, mask=cyan_mask)

    # red_mask = cv2.inRange(hsv, low_red, high_red)
    # red = cv2.bitwise_and(frame, frame, mask=red_mask)

    # convert and invert to grayscale for blob detection
    gray = cv2.cvtColor(cyan, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    img_binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    # detect blobs
    keypoints = detector.detect(img_binary)

    # RESET tracking with w key
    if cv2.waitKey(1) & 0xFF == ord('w'):
        print("tracking reset")
        pts.clear()
        counter = 0
        initialized = False

    # only proceed if at least one blob was found
    b = None
    if len(keypoints) > 0:
        # find the largest blob in the mask
        b = max(keypoints, key=get_size)
        diameter = b.size
        (x, y) = b.pt
        pt = (int(x), int(y))

        # only proceed if the radius meets a minimum size
        if diameter > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(diameter / 2), (0, 255, 255), 2)
            # cv2.circle(frame, None, 5, (0, 0, 255), -1)

        # dont add the same point twice
        if pt == last_pt and counter > 0:
            pass
        else:
            # print(pt, counter)
            pts.appendleft(pt)
            counter += 1
            last_pt = pt

    # after the ball has been seen twice, estimate initial x,y pos and velocity, initialize kalman filter position
    if counter == 2 and initialized is False:
        (x1, y1) = pts.pop()
        (x2, y2) = pts.pop()
        # estimate initial state from 2 frames
        x_0 = np.array([[x2], [(290 - y2)], [(x2 - x1) / dt / 8], [(y1 - y2) / dt / 8]]) / pix_per_meter
        print("initial position")
        print(x_0)
        f1.x = x_0
        initialized = True

    if initialized is True:
        # make Kalman state prediction
        f1.predict(u=g)
        # see if there is an available measurement
        try:
            # convert sensor reading to meters for z matrix for Kalman filter
            reading = pts.pop()
            z[0][0] = reading[0] / pix_per_meter
            z[1][0] = (290 - reading[1]) / pix_per_meter
            f1.update(z)
        except:
            pass

        # Predict when the ball will enter the interception area
        # send angle over serial to ev3
        img_binary, f1.x = get_interception(img_binary)

    im_with_keypoints = cv2.drawKeypoints(img_binary, [b], np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow('frame', im_with_keypoints)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # cv2.imwrite('test2.jpg', bgr_inv)
        # cv2.imwrite('test3.jpg', frame)
        break

    # end_time = time.time()
    # time_counter += 1
    # running_average += (end_time - start_time)
    # print(running_average / time_counter)
    # start_time = end_time

# close bluetooth serial bus -- IMPORTANT
client_socket.close()
cv2.destroyAllWindows()
cap.release()
