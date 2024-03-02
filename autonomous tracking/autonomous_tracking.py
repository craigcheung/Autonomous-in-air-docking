import cv2
import numpy as np
import math
import time
import butterworthfilter_module as bw
from pymavlink import mavutil
#pip install pymavlink
#mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600
#Adjust the --master parameter to match your Pixhawk's USB port.


def image_processing(frame):
    kern = 51 # blur kernal size
    blur = cv2.GaussianBlur(frame,(kern,kern),0)
    colors = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) 
    mask, result = mask_red(frame)
    _, thresh = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)
    thresh = cv2.convertScaleAbs(thresh)
    contours,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    frame_w_lines = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    return blur, mask, frame_w_lines, contours

def mask_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 65, 60]) # hue, saturation, brightness
    upper_red1 = np.array([20, 255, 255])
    lower_red2 = np.array([160, 65, 60])
    upper_red2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, result

def compare_centroid_to_avg(img_cent, imgxMax, contours, avg):
    closest_dist = imgxMax
    closest_cent = [0,0]
    for c in contours:
        M = cv2.moments(c) # dictionary of moments
        if (M['m00'] != 0): # prevents divide-by-zero errors
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            img_cent = cv2.circle(img_cent, (cx,cy), radius=20, color=(255,0,0), thickness=2) # centorid of contour (blue circle)
            dist = np.sqrt((cx-avg[0])**2 + (cy-avg[1])**2) # disance from overall avg to centroid
            if dist < closest_dist:
                closest_dist = dist # replace old value with the new closest distance
                closest_cent = [cx,cy] # centroid closest to overall average
    if closest_cent == [0,0]:
        closest_cent = avg
    img_cent = cv2.circle(img_cent, closest_cent, radius=20, color=(0,255,255), thickness=2) # centorid closest to avg (yellow circle)
    return closest_cent, img_cent

def calc_phys_dist(centroid, imgx, imgy, mask, focalLengthTimesWidth, filterPixelCount):
    distance_in_meters = calculate_distance(focalLengthTimesWidth, filterPixelCount, imgx)
    centx = centroid[0]
    centy = centroid[1]
    xMeters, yMeters = convert_pixels_to_meters(centx, centy, distance_in_meters)
    return xMeters, yMeters, distance_in_meters

def calulate_overall_mean(mask, imgx, imgy):
    indices = np.where(mask > 150)
    filterPixelCount = len(indices[0]) + 1.0
    sumCordinateXValue = np.sum(indices[1])
    sumCordinateYValue = np.sum(indices[0])
    avg_x = (sumCordinateXValue / filterPixelCount).astype(int)
    avg_y = (sumCordinateYValue / filterPixelCount).astype(int)
    avg = [avg_x, avg_y]
    return avg, filterPixelCount

def calculate_distance(focalLengthTimesWidth, filterPixelCount, imgx):
    distance_in_meters = focalLengthTimesWidth / (math.sqrt(filterPixelCount) / imgx)
    #distance_in_inches = distance_in_meters * 39.37
    return distance_in_meters

#check math
def convert_pixels_to_meters(outputx, outputy, distance_in_meters):
    thetaX = 0.698 #40 degrees
    thetaY = 0.393 #22.5 degrees
    xMeters = ((-2.0 * (outputx - 0.5)) * distance_in_meters * math.tan(thetaX))
    yMeters = ((2.0 *(outputy -0.5)) * distance_in_meters * math.tan(thetaY))
    return xMeters, yMeters

def pymavlink_data_send(yawInput, vel_z, pos_x, pos_y, pos_z, master):
    
    #mode = 0b110111000111 #velocity control
    #mode = 0b110111000000 #pos + velo control
    #mode = 0b110111111000 # position controL
    mode = 0b100111000000 #pos + velo control + yaw maybe remove first 1.
    
    msg = master.mav.set_position_target_local_ned_encode(
        time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
        target_system=1,  # Replace with your Pixhawk system ID
        target_component=mavutil.mavlink.MAV_COMP_ID_ALL,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_FRD, ##Define MAV_FRAME_LOCAL_NED as position rel origin
        ##DEFINE MAV_FRAME_LOCAL_OFFSET_NED as position relative to current... MAV_FRAME_LOCAL_OFFSET_NED #body should be relative
        type_mask= mode,  # Position type mask
        x=pos_x,
        y=pos_y,
        z=0,
        vx=0.2,
        vy=0.2,
        vz=0.2,
        afx=0.2,
        afy=0.05,
        afz=0.05,
        yaw = 0,
        yaw_rate= 0.15
    )

    master.mav.send(msg)
    return 0


def main():
    #ttyUSB0 is radio
    #ttyACM0 is direct connection with usb
    comPort = '/dev/ttyUSB0' #COM for windows.  for linux /dev/ttyUSB0
    baudRate = '57600' #115200 for usb 57600 for telem
    master = mavutil.mavlink_connection(comPort, baud=baudRate)

    #data logging and video settings
    cap = cv2.VideoCapture(0)
    filename = "test"
    f = open(filename + ".txt", "w")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    imgxMax = 720
    imgx, imgy = imgxMax, imgxMax
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (imgx, imgy)) 

    #variables, change to actual for drone
    focalLengthTimesWidth = 0.086
    distance_in_meters = 100
    cent_signal = []
    SIGNAL_DURATION = 10
    init_time = time.time()
    relative_position_adjustment_x = 0
    relative_position_adjustment_y = 0
    try:
        while distance_in_meters > 0:
            timeStart = time.time()
            ret, frame = cap.read()
            frame = cv2.flip(frame, 1)
            if not ret:
                f.write("Video error")
                break
            frame = cv2.resize(frame, (imgx, imgy))
            blur, mask, frame_w_lines, contours = image_processing(frame) # Image Processing
            avg, filterPixelCount = calulate_overall_mean(mask, imgx, imgy) # Overall Average
            img_cent = cv2.circle(frame_w_lines, (avg), radius=20, color=(0,0,255), thickness=2) # draw overall average (red circle)
            closest_cent, img_cent = compare_centroid_to_avg(img_cent, imgxMax, contours, avg) # Centroids
            xMeters, yMeters, dist_m = calc_phys_dist(closest_cent, imgx, imgy, mask, focalLengthTimesWidth, filterPixelCount) # Calculates Distances Using Centroid
            
            #cv2.imshow('mask', mask)
            #cv2.imshow('centroid', img_cent)
                       
            if (filterPixelCount < 100):
                closest_cent = [0,0,100]
                velocity_z = -0.1
            else:
                relative_position_adjustment_x = xMeters
                relative_position_adjustment_y = yMeters
                #velocity_z = 0

            cv2.waitKey(1)
            timeEnd = time.time()
            timeLoop = timeEnd - timeStart
            
            
            if (len(cent_signal) < SIGNAL_DURATION):
                closest_cent.append(0)
                cent_signal.append(closest_cent)
            else:
                # not sure if filtered data point should be appended back to signal, because you are refiltering already
                # filtered data
                cent_signal.pop(0)
                closest_cent.append(dist_m)
                cent_signal.append(closest_cent)
                closest_cent = bw.filter_data(cent_signal, "test")
            
            # implement control loops for center tracking
            # feed values into pymavlink_data_send, you can change the code as needed for messages sent
            pymavlink_data_send(0, -0.1, closest_cent[0], closest_cent[1], dist_m, master)
            print(str(round(time.time() - init_time, 2)) + " " + str(round(closest_cent[0],2)) + " "+ str(round(closest_cent[1],2)) +
                " " + str(round(dist_m,2)) + "\n")
            f.write(str(round(time.time() - init_time, 2)) + " " + str(round(relative_position_adjustment_x,2)) + " "+ str(round(relative_position_adjustment_y,2)) +
                " " + str(round(dist_m,2)) + "\n")
            out.write(frame)
            
            #print(timeLoop);
            #print(relative_position_adjustment_y, "relative y")
            #print(velocity_z, "velocity z")
        f.write("program terminated")
        f.close()
        cap.release()
        out.release()
    except KeyboardInterrupt:
        f.write("user interrupted")
        cap.release()
        out.release()
        f.close()

if __name__ == "__main__":
    main()
