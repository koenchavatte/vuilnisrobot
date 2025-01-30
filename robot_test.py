# With help of ChatGPT 4o
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from Motor import Motor
from Ultrasonic import Ultrasonic
from servo import Servo

motor = Motor()
ultrasonic = Ultrasonic()
servo = Servo()
picam2 = Picamera2()

picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

LOWER_DARK_BLUE = np.array([100, 150, 50])
UPPER_DARK_BLUE = np.array([130, 255, 150])

LOWER_BRIGHT_BLUE = np.array([100, 50, 200])
UPPER_BRIGHT_BLUE = np.array([130, 255, 255])


GRIPPER_OPEN = 90
GRIPPER_CLOSED = 150
ARM_DOWN = 90
ARM_UP = 140

def detect_blue_object():
    frame = picam2.capture_array()
    frame = cv2.flip(frame, -1)  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_dark = cv2.inRange(hsv, LOWER_DARK_BLUE, UPPER_DARK_BLUE)
    mask_bright = cv2.inRange(hsv, LOWER_BRIGHT_BLUE, UPPER_BRIGHT_BLUE)
    mask = mask_dark + mask_bright

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        if w * h > 1000:
            print(f"Blue object detected at x={x}, width={w}, height={h}")
            return True, (x, y, w, h)
    
    print("No blue object detected.")
    return False, None

def avoid_obstacles():
    print("Avoiding obstacle...")
    motor.setMotorModel(-1500, -1500)
    time.sleep(0.5)

    left_distance = ultrasonic.get_distance()
    time.sleep(0.2)
    
    motor.setMotorModel(0, 0)  

    if left_distance > 20:
        print("Turning left to avoid obstacle.")
        motor.setMotorModel(-1500, 1500)
        time.sleep(0.6)
    else:
        print("Turning right to avoid obstacle.")
        motor.setMotorModel(1500, -1500)
        time.sleep(0.6)

    motor.setMotorModel(0, 0)
    time.sleep(0.5)

def patrol_and_search():
    print("Searching for the object...")
    motor.setMotorModel(1500, 1500)
    
    for _ in range(10):
        distance = ultrasonic.get_distance()
        found, _ = detect_blue_object()

        if found:
            print("Object found during patrol!")
            return True

        if distance < 15:
            avoid_obstacles()

        time.sleep(0.3)

    motor.setMotorModel(1500, -1500)
    time.sleep(0.8)

    return False

def align_with_blue_object(bbox, frame_width, distance):
    x, _, w, _ = bbox
    center_x = x + w // 2
    frame_center = frame_width // 2
    tolerance = 30  

    print(f"Alignment: CenterX={center_x}, Frame Center={frame_center}, Object Width={w}, Distance={distance:.2f} cm")

    if abs(center_x - frame_center) < 20:
        print("Object is now perfectly centered!")
        return True  

    if center_x < frame_center:
        turn_direction = (-1, 1)  
        print("Turning LEFT to align.")
    else:
        turn_direction = (1, -1)  
        print("Turning RIGHT to align.")

    speed = 1050
    turn_time = 0.1
    motor.setMotorModel(speed * turn_direction[0], speed * turn_direction[1])
    time.sleep(turn_time)

    return False  

def pick_up_object():
    print("Lowering arm to pick up object...")
    for i in range(140, 90, -1):  
        servo.setServoPwm('1', i)
        time.sleep(0.02)

    print("Closing gripper...")
    for i in range(120, 150, 1):
        servo.setServoPwm('0', i)
        time.sleep(0.02)

    print("Lifting object...")
    for i in range(90, 140, 1):  
        servo.setServoPwm('1', i)
        time.sleep(0.02)

    print("Object is secured!")

def drop_off_object():
    print("Opening gripper...")
    for i in range(150, 90, -1):  
        servo.setServoPwm('0', i)
        time.sleep(0.02)

    print("Object has been released!")

    print("Drop-off complete!")

    print("Resuming patrol...")
    patrol_and_search()

LOWER_FLUO_GREEN = np.array([35, 120, 80])
UPPER_FLUO_GREEN = np.array([90, 255, 255])

def detect_green_box():
    frame = picam2.capture_array()
    frame = cv2.flip(frame, -1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, LOWER_FLUO_GREEN, UPPER_FLUO_GREEN)

    kernel = np.ones((5, 5), np.uint8)
    mask_green = cv2.dilate(mask_green, kernel, iterations=2)

    contours, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 200:
            x, y, w, h = cv2.boundingRect(contour)
            print(f"Green box detected at x={x}, y={y}, width={w}, height={h}, area={area}")
            return True, (x, y, w, h)

    return False, None

def scan_for_green_box_360():
    print("Performing a slow 360-degree scan for the green box...")
    
    for _ in range(20):
        motor.setMotorModel(1150, -1150)
        time.sleep(0.2)

        found, bbox = detect_green_box()
        if found:
            print("Green box found during 360-degree scan!")
            motor.setMotorModel(0, 0)
            return bbox

    motor.setMotorModel(0, 0)
    print("No green box found in 360-degree scan.")
    return None

def search_for_green_box():
    print("Searching for green box while moving forward...")
    motor.setMotorModel(1200, 1200)

    for _ in range(15):
        distance = ultrasonic.get_distance()
        found, bbox = detect_green_box()

        if found:
            print("? Green box found while moving forward!")
            return bbox

        if distance < 15:
            print("?? Obstacle detected while searching for the green box!")
            avoid_obstacles()

        time.sleep(0.3)

    motor.setMotorModel(1500, -1500)
    time.sleep(0.8)

    return None


def align_with_green_box(bbox, frame_width):
    x, _, w, _ = bbox
    center_x = x + w // 2
    frame_center = frame_width // 2
    tolerance = 30

    print(f"Aligning: Box Center={center_x}, Frame Center={frame_center}")

    if abs(center_x - frame_center) < tolerance:
        print("? Green box is centered! Ready to move forward.")
        return True

    if center_x < frame_center:
        turn_direction = (-1, 1)
        print("?? Turning LEFT to align with the green box.")
    else:
        turn_direction = (1, -1)
        print("?? Turning RIGHT to align with the green box.")

    misalignment = abs(center_x - frame_center)
    if misalignment > 100:
        speed = 1100
        turn_time = 0.15
    elif misalignment > 50:
        speed = 1050
        turn_time = 0.1
    else:
        speed = 1000
        turn_time = 0.05

    motor.setMotorModel(speed * turn_direction[0], speed * turn_direction[1])
    time.sleep(turn_time)

    return False

def approach_green_box():
    while True:
        distance = ultrasonic.get_distance()

        found, bbox = detect_green_box()
        
        if found:
            print("? Green box detected! Approaching...")
        else:
            print("? No green box detected. Searching...")

        if 3.5 <= distance <= 4.5:
            print(f"? Correct distance reached: {distance:.2f} cm. Dropping off object.")
            motor.setMotorModel(0, 0)
            time.sleep(0.5)
            return

        if distance < 3.5:
            print(f"?? Overshot! Distance is {distance:.2f} cm. Reversing slightly...")
            motor.setMotorModel(-1000, -1000)
            time.sleep(0.2)
            continue

        if found and bbox is not None:
            aligned = align_with_green_box(bbox, 640)
            if not aligned:
                continue

        if not found and distance < 15:
            print("?? Obstacle detected while approaching, but it's NOT the green box! Avoiding...")
            avoid_obstacles()
            continue

        if distance > 10:
            motor.setMotorModel(1050, 1050)
        else:
            motor.setMotorModel(1000, 1000)
        
        time.sleep(0.2)


def main():
    while True:
        distance = ultrasonic.get_distance()
        if distance < 5 or distance > 400:
            print("Ultrasonic sensor returned an invalid reading. Ignoring...")
            continue

        print(f"Distance: {distance:.2f} cm")

        found, bbox = detect_blue_object()
        if not found:
            if patrol_and_search():
                continue
            else:
                print("No object found, resuming patrol...")
                continue

        print("Blue object detected! Approaching...")

        while True:
            if bbox is None:
                print("Lost blue object! Re-scanning...")
                for _ in range(3):
                    found, bbox = detect_blue_object()
                    if found:
                        break
                    time.sleep(0.2)

                if bbox is None:
                    print("Object lost completely. Resuming patrol...")
                    motor.setMotorModel(0, 0)
                    time.sleep(1)
                    patrol_and_search()
                    continue

            if align_with_blue_object(bbox, 640, distance):
                break

            distance = ultrasonic.get_distance()
            found, bbox = detect_blue_object()

        print("Object aligned! Moving toward it.")

        while True:
            distance = ultrasonic.get_distance()
            
            if 6.8 <= distance <= 7.2:  
                print(f"Perfect distance reached: {distance:.2f} cm. Picking up object.")
                motor.setMotorModel(0, 0)
                time.sleep(0.5)
                break

            if distance < 6.8:
                print(f"Overshot! Distance is {distance:.2f} cm. Reversing slightly...")
                motor.setMotorModel(-1000, -1000)  
                time.sleep(0.2)  
                continue

            found, bbox = detect_blue_object()
            if found and bbox is not None:
                align_with_blue_object(bbox, 640, distance)

            if distance > 20:
                motor.setMotorModel(1100, 1100)  
            elif distance > 15:
                motor.setMotorModel(1050, 1050)  
            else:
                motor.setMotorModel(1000, 1000)  
            
            time.sleep(0.2)

        pick_up_object()

        green_bbox = scan_for_green_box_360()
        
        if green_bbox:
            print("Green box found! Moving toward drop-off location.")
        else:
            print("No green box found in scan. Proceeding with normal search.")
            green_bbox = search_for_green_box()

        if green_bbox:
            print("Approaching green box for drop-off.")
            approach_green_box()
            drop_off_object()
        else:
            print("Green box not found. Continuing patrol.")

        print("Resuming patrol...")
        motor.setMotorModel(0, 0)
        time.sleep(1)
        motor.setMotorModel(1500, 1500)


try:
    main()
except KeyboardInterrupt:
    print("\nStopping program. Resetting servos to start position.")
    motor.setMotorModel(0, 0)
    servo.setServoPwm('0', GRIPPER_OPEN)
    servo.setServoPwm('1', ARM_UP)
    print("Servos reset. Program Stopped.")
