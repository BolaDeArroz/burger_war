
import cv2
import copy
import numpy as np



# create sample mask image window function-------------------------------------------------
def createMaskImage(hsv, hue, sat, val):
    imh, imw, channels = hsv.shape  # get image size and the number of channels
    mask = np.zeros((imh, imw, channels), np.uint8) # initialize hsv gradation image with 0

    # if hue argument is pair value enclosed in []
    hmin = hue[0]
    hmax = hue[1]

    # if sat argument is pair value enclosed in []
    smin = sat[0]
    smax = sat[1]

    #  val argument is pair value enclosed in []
    vmin = val[0]
    vmax = val[1]

    return cv2.inRange(hsv, np.array([hmin, smin, vmin]), np.array([hmax, smax, vmax]))

def detect_enemy_robot(frame):
    result_dict = {}
    img = frame

    # convert to HSV (Hue, Saturation, Value(Brightness))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("Hue", hsv[:, :, 0])
    """
    use the bgr method at gazebo because hsv not show
    """
    # find red ball
    # bgr
    rnb_red = createMaskImage(img, [0 , 10], [0, 10], [100, 255])
    # METHOD1: fill hole in the object using Closing process (Dilation next to Erosion) of mathematical morphology
    rnb_red = cv2.morphologyEx(rnb_red, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
    rnb_red = cv2.morphologyEx(rnb_red, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9)))
    cv2.imshow("rnb_red", rnb_red)
    im, contours, hierarchy = cv2.findContours(rnb_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Draw_txt(im, contours, hierarchy,img)
    draw_obj_label(contours, "red", img, "red")
    
    result_dict['red_ball'] = contours
    #rnb_green = createMaskImage(hsv, [110 ,130], [80, 100], [40, 60])
    # bgr
    rnb_green = createMaskImage(img, [0 , 10], [140, 160], [0, 10])
    # METHOD1: fill hole in the object using Closing process (Dilation next to Erosion) of mathematical morphology
    rnb_green = cv2.morphologyEx(rnb_green, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))) # using 9x9 ellipse kernel
    rnb_green = cv2.morphologyEx(rnb_green, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,4))) # using 4x4 ellipse kernel
    cv2.imshow("rnb_green", rnb_green)
    im, contours, hierarchy = cv2.findContours(rnb_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Draw_txt(im, contours, hierarchy,img)
    draw_obj_label(contours, "green", img, "green")
    result_dict['green_side'] = contours

    # burger
    rnb_burger = createMaskImage(img, [25 , 27], [25, 27], [25, 27])
    rnb_burger = cv2.morphologyEx(rnb_burger, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2)))
    rnb_burger = cv2.morphologyEx(rnb_burger, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(39,39)))
    cv2.imshow("rnb_burger", rnb_burger)
    im, contours, hierarchy = cv2.findContours(rnb_burger, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Draw_txt(im, contours, hierarchy,img)
    draw_obj_label(contours, "almond", img, "almond")
    result_dict['burger'] = contours
    return result_dict

    


def calc_enemy_center(contours):
    maxCont=contours[0]
    for c in contours:
        if len(maxCont)<len(c):
            maxCont=c
    mu = cv2.moments(maxCont)
    if mu["m00"] == 0:
        mu["m00"] = 0.1
    x,y= int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
    return [x, y]


def calc_enemy_area(contours):
    max_area = 0
    for i, cnt in enumerate(contours):
        # calculate area
        area = cv2.contourArea(cnt)
        if max_area < area:
            max_area = area
    return max_area



def Draw_txt(im, contours, hierarchy,img):
    for i in range(len(contours)):
        # -- get information of bounding rect of each contours
        posx, posy, width, height = cv2.boundingRect(contours[i])
        # -- decide "Skal" object using aspect ratio of bounding area
        if width*2<height and height<width*6: # --
            cv2.rectangle(img, (posx, posy), (posx + width, posy + height), (0, 0, 255), 2)
            strSize = cv2.getTextSize("Grande Cylinder", cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.rectangle(img, (posx, posy-strSize[1]), (posx+strSize[0], posy), (0, 0, 255), -1)
            cv2.putText(img, "Another Object", (posx, posy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
        else: # -- is not "Skal"
            cv2.rectangle(img, (posx, posy), (posx + width, posy + height), (0, 255, 0), 2)
            cv2.imshow("Image", img)



def draw_obj_label(contours,label_data, target_img, color):
    grande_aspect_ratio_min = 1/6
    grande_aspect_ratio_max = 1/2
    if(color == "green"):
        r = 0
        g = 255
        b = 0
    elif(color == "almond"):
        r = 255
        g = 255
        b = 0
    elif(color == "pink"):
        r = 229
        g = 152
        b = 197
    elif(color == "red"):
        r = 255
        g = 0
        b = 0
    else:
        r=255
        g=255
        b=255

    for i in range(len(contours)):
        # -- get information of bounding rect of each contours
        posx, posy, width, height = cv2.boundingRect(contours[i])
        # -- decide "Skal" object using aspect ratio of bounding area
        if grande_aspect_ratio_min < width/height and width / height < grande_aspect_ratio_max: # --
            cv2.rectangle(target_img, (posx, posy), (posx + width, posy + height), (0, 0, 255), 2)
            mesg_list=[]
            mesg_list.append(label_data+" grande")
            aspect_ratio = width / height
            mesg_list.append(int(100*aspect_ratio))  ### aspect ratio
            # 0.6 is text size
            draw_text_red(posx,posy,0.6,mesg_list,target_img)
        else:
            cv2.rectangle(target_img, (posx, posy), (posx + width, posy + height), (b, g, r), 2)
            mesg_list = []
            mesg_list.append(label_data )
            aspect_ratio = width / height
            mesg_list.append(int(100 * aspect_ratio))  ### aspect ratio
            # 0.6 is text size
            draw_text_color(posx, posy, 0.6, mesg_list, target_img, color)


def draw_text_color(x,y,size,mesg_list,target_img, color):

    if(color == "green"):
        r = 0
        g = 255
        b = 0
    elif(color == "almond"):
        r = 255
        g = 255
        b = 0
    elif(color == "pink"):
        r = 229
        g = 152
        b = 197
    elif(color == "red"):
        r = 255
        g = 0
        b = 0
    else:
        r=255
        g=255
        b=255

    for mesg in mesg_list:
        strSize = cv2.getTextSize(str(mesg), cv2.FONT_HERSHEY_SIMPLEX, size, 1)[0]
        cv2.rectangle(target_img, (x, y - strSize[1]), (x + strSize[0], y), (b, g,r), -1)
        cv2.putText(target_img, str(mesg), (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0))
        y=y+int(strSize[1]*12/10)



def get_tracking_info(original_image):
    result_dict = {}
    enemy_center = 0
    enemy_area = 0
    # get contours from enemy
    enemy_robot_contours = detect_enemy_robot(original_image)
    cv2.imshow("Image", original_image)
    cv2.waitKey(1)

    red_ball_contours = enemy_robot_contours['red_ball']
    green_side_contours = enemy_robot_contours['green_side']
    burger_contours = enemy_robot_contours['burger']
    if red_ball_contours != []:
        enemy_center = calc_enemy_center(red_ball_contours)
        enemy_area = calc_enemy_area(red_ball_contours)
        # outside of tracking area 
        if enemy_area < 900:
            """
            you can write navigation code using enemy coordinate
            """
            return result_dict
        else:
            result_dict['center'] = enemy_center
            result_dict['enemy_area'] = enemy_area
            result_dict['target'] = 'red_ball'
            return result_dict
    elif green_side_contours != []:
        enemy_center = calc_enemy_center(green_side_contours)
        enemy_area = calc_enemy_area(green_side_contours)
        result_dict['center'] = enemy_center
        result_dict['enemy_area'] = enemy_area
        result_dict['target'] = 'green_side'
        return result_dict
    elif burger_contours != []:
        enemy_center = calc_enemy_center(burger_contours)
        enemy_area = calc_enemy_area(burger_contours)
        result_dict['center'] = enemy_center
        result_dict['enemy_area'] = enemy_area
        result_dict['target'] = 'burger'
        return result_dict
    return result_dict