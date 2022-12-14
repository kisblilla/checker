from robodk.robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import cv2
import numpy as np

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()
PATH='.'
CROP_START = 367
PIXELPERMM = 40/23

# Select a robot (popup is displayed if more than one robot is available)
# gather robot, tool and reference frames from the station
robot               = RDK.Item('', ITEM_TYPE_ROBOT)
tool                = RDK.Item('', ITEM_TYPE_TOOL)
table               = RDK.Item('', ITEM_TYPE_FRAME)
if not robot.Valid():
    raise Exception('No robot selected or available')

target_ref = robot.Pose()
pos_ref = target_ref.Pos()
ref_image = [338, 88]
ref_real = [140, 400]

# It is important to provide the reference frame and the tool frames when generating programs offline
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(tool)
#robot.setRounding(10)  # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
robot.setSpeed(10)  # Set linear speed in mm/s
robot.setAcceleration(-1)
robot.setAccelerationJoints(-1)


def TCP_On(toolitem):
    """Attaches the closest object to the toolitem Htool pose,
    It will also output appropriate function calls on the generated robot program (call to TCP_On)"""
    toolitem.AttachClosest()
    #toolitem.RDK().RunMessage('Set air valve on')
    toolitem.RDK().RunProgram('TCP_On()')
        
def TCP_Off(toolitem, itemleave=0):
    """Detaches the closest object attached to the toolitem Htool pose,
    It will also output appropriate function calls on the generated robot program (call to TCP_Off)"""
    #toolitem.DetachClosest(itemleave)
    toolitem.DetachAll(itemleave)
    #toolitem.RDK().RunMessage('Set air valve off')
    toolitem.RDK().RunProgram('TCP_Off()')

def zoom(img, zoom_factor=2):
    return cv2.resize(img, None, fx=zoom_factor, fy=zoom_factor)

def detect_checkers(path):
    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    image = cv2.imread(path)
    # Convert to grayscale.
    print(image.shape)
    cv2.imshow('img', image)
    cv2.waitKey(0)
    
    cropped = image[:, CROP_START:489]
    zoomed_and_cropped = zoom(cropped, 3)
    gray = cv2.cvtColor(zoomed_and_cropped, cv2.COLOR_BGR2GRAY)
    print(gray.shape)
    # Blur using 3 * 3 kernel.
    gray_blurred = gray
    cv2.imshow('img', gray_blurred)
    cv2.waitKey(0)
    detected_circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 9, 5, minRadius=10, maxRadius=40)
    checker_position_list = []

    # Draw circles that are detected.
    # Convert the circle parameters a, b and r to integers.
    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))
        checker_position_list = detected_circles
        print("checker_position_list", checker_position_list)
        colorized = cv2.cvtColor(gray_blurred, cv2.COLOR_GRAY2BGR)  
        colors = []
        for i in detected_circles[0,:]:
            print(f"detected circle: ", "x:", i[0],"y",i[1])
            
            if (colorized[i[1],i[0]][0] == 255):
                print("white")
                colors.append("white")
                color = (51, 224, 176)
                cv2.circle(colorized, (i[0], i[1]), i[2], color, 3)
            else: 
                print("black")
                colors.append("black")
                color = (233, 150, 122)
                cv2.circle(colorized, (i[0], i[1]), i[2], color, 3)
            #print(colorized[i[1],i[0]])
            # Show result for testing:
            cv2.imshow('img', colorized)
            cv2.waitKey(0)

    else:
        print("There is no circle in this area.")
        exit()
    return checker_position_list, colors

                            

def WaitPartCamera():
    """Simulate camera detection"""
    if RDK.RunMode() == RUNMODE_SIMULATE:
        #Use real image processing and brick detection
        print("Saving camera snapshot to file:" + f'{PATH}/tmp.png')            
        RDK.Cam2D_Snapshot(f'{PATH}/tmp.png')
        # Implement your image processing here:
        return detect_checkers(f'{PATH}/tmp.png')
    else:
        RDK.RunProgram('WaitPartCamera')
    return 0,0,0



def calculate_real_position(x,y):
    gray_img_x = x/3+CROP_START
    gray_img_y = y/3
    dif_x = (gray_img_x - ref_image[0])*PIXELPERMM
    dif_y = (gray_img_y - ref_image[1])*PIXELPERMM
    real_x = dif_x+ref_real[0]
    real_y = dif_y+ref_real[1]
    return real_x, real_y

def pickup_checker(target,lego_brick_list, colors):
    print("len(lego_brick_list)", len(lego_brick_list))
    idx_white = 40
    idx_black = 40
    robot.MoveJ(Pose(46.458,197.957, 313.628,177.085, 1.908, 4.471))
    robot.MoveJ(Pose(59.315,362.264, 305.691, 177.085, 1.908, 4.471))
    robot.MoveJ(Pose(63.810, 355.403, 170.940, 177.085, 1.908, 4.471))
    for i in range(len(lego_brick_list)):
        print("len(lego_brick_list)", len(lego_brick_list[i]))
        for j in range(len(lego_brick_list[i])):
            x = lego_brick_list[i][j][0]
            y = lego_brick_list[i][j][1]
            r = lego_brick_list[i][j][2]
            x,y =calculate_real_position(x,y)
            if colors[j] == "white":
                robot.MoveJ(Pose(288.001, 338.265, 179.292, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(291.997, 332.166, 59.514, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(506.224, 315.789, 67.494, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(y,x, 5,  177.085, 1.908, 4.471))
                TCP_On(tool)

                robot.MoveJ(Pose(370, -140+idx_white, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(370, -140+idx_white, 82.085, 177.085, 1.908, 4.471))
                TCP_Off(tool)
                robot.MoveJ(Pose(370, -140+idx_white, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(370, 100, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(370, 100, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(288.001, 280, 179.292, 177.085, 1.908, 4.471))
                            
                idx_white = idx_white + 80
            else:
                robot.MoveJ(Pose(288.001, 338.265, 179.292, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(291.997, 332.166, 59.514, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(506.224, 315.789, 67.494, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(y,x, 5,  177.085, 1.908, 4.471))
                TCP_On(tool)

                robot.MoveJ(Pose(640, -140+idx_black, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(640, -140+idx_black, 82.085, 177.085, 1.908, 4.471))
                TCP_Off(tool)
                robot.MoveJ(Pose(640, -140+idx_black, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(640, 100, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(640, 100, 280.206, 177.085, 1.908, 4.471))
                robot.MoveJ(Pose(288.001, 280, 179.292, 177.085, 1.908, 4.471))
                            
                idx_black = idx_black + 80


lego_brick_list, colors = WaitPartCamera()
target = RDK.Item('Target')  
pickup_checker(target, lego_brick_list, colors)
print('Done')