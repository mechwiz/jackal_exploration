#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler

global costmap
global flagm, flagg
global init
global newcost
global grid
global feedback
global prevpnt
global sample
global stuckpnt
global offsetcnt
global firstgoal
global checkpnt

sample = []
newcost = []
flagm = 0
flagg = 0
init = 0
prevpnt = 0
offsetcnt = 0
stuckpnt = []
firstgoal=0
checkpnt = 0

cost_update = OccupancyGridUpdate()
costmap = OccupancyGrid()
grid = OccupancyGrid()
feedback = MoveBaseActionFeedback()
result = MoveBaseActionResult()

newPose = PoseStamped()
q = Quaternion()

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=30)


def costupCb(data):
    global flagg, flagm
    global init
    global newcost
    flagg = 1
    cost_update = data

    if init == 1 and flagm == 0:
        gpnts = cost_update.data
        gwidth = cost_update.width
        gheight = cost_update.height
        gx = cost_update.x
        gy = cost_update.y
        cnt = 0
        for x in range(gheight):
            for y in range(gwidth):
                g = gpnts[cnt]
                newcost[x+gy][y+gx] = g
                cnt += 1

def costCb(data):
    global costmap
    global flagm
    global newcost

    newcost = []
    flagm = 1
    costmap = data

    width = costmap.info.width
    height = costmap.info.height
    pnts = costmap.data
    cnt = 0

    for x in range(height):
        newcost_x = []
        for y in range(width):
            newcost_x.append(pnts[cnt])
            cnt+=1
        newcost.append(newcost_x)

def mapCb(data):
    global costmap
    global flagm
    global init
    global grid
    grid = data

    if flagm == 1:
        flagm = 0
        init = 1
        rospy.loginfo('received1')
        res = grid.info.resolution
        width = grid.info.width
        height = grid.info.height
        pnts = grid.data
        gpnts = costmap.data
        cnt = 0
        pic_mat = []

        for x in range(height):
            pic_x = []
            for y in range(width):
                p = pnts[cnt]
                g = gpnts[cnt]

                if (g>0):
                    p = 0
                else:
                    if (p ==-1):
                        p = 50
                    elif (p == 0):
                        p = 100
                    else:
                        p = 0
                pic_x.append(p)
                cnt += 1
            pic_mat.append(pic_x)

        # plt.imshow(pic_mat,cmap='gray',origin='lower')
        # plt.draw()
        # plt.pause(0.001)

def feedbackCb(data):
    global feedback
    global sample
    global stuckpnt
    global offsetcnt
    global prevpnt
    global firstgoal
    global checkpnt

    offsetx = [1,-1,1,-1]
    offsety = [1,-1,-1,1]

    feedback = data
    x = feedback.feedback.base_position.pose.position.x
    y = feedback.feedback.base_position.pose.position.y
    sample.append([x,y])

    if firstgoal == 0:
        sendGoal([x+0.5,y+0.5])
        firstgoal = 1

    if len(sample) > 15:
        sa = np.array(sample)
        mx = np.average(sa[:,0]) - x
        my = np.average(sa[:,1]) - y
        if abs(mx)<0.001 and abs(my)<0.001:
            if checkpnt < 3:
                prevpnt = 0
            rospy.loginfo('Stuck. Resetting Goal...')
            if len(stuckpnt) > 0:
                sx = stuckpnt[0]
                sy = stuckpnt[1]
                if abs(sx-x) < 0.001 and abs(sy-y) < 0.001:
                    rospy.loginfo("Changing Point")
                    x = x + 0.5*offsetx[offsetcnt]
                    y = y + 0.5*offsety[offsetcnt]
                    offsetcnt += 1
                    if offsetcnt > 3:
                        offsetcnt = 0
            sendGoal([x,y])
            stuckpnt = [x,y]
        sample = []

def sendNavCb(data):
    global grid
    global newcost
    global init
    global flagg
    global feedback
    global prevpnt
    global sample
    global checkpnt

    sample = []
    send = True
    result = data
    frontier_mat = []
    if result.status.text == "Goal reached.":
        if flagg == 1 and init == 1:
            flagg = 0
            rospy.loginfo('received2')
            res = grid.info.resolution
            width = grid.info.width
            height = grid.info.height
            pnts = grid.data
            gpnts = np.array(newcost).flatten().tolist()
            cnt = 0
            pic_mat = []

            for x in range(height):
                pic_x = []
                for y in range(width):
                    p = pnts[cnt]
                    g = gpnts[cnt]

                    if (g>0):
                        p = 0
                    else:
                        if (p ==-1):
                            p = 50
                        elif (p == 0):
                            p = 100
                        else:
                            p = 0
                    pic_x.append(p)
                    cnt += 1
                pic_mat.append(pic_x)

            ox = grid.info.origin.position.x
            oy = grid.info.origin.position.y
            mc = interp1d([ox,ox+width*res],[0,width],bounds_error=False)
            mr = interp1d([oy,oy+height*res],[0,height],bounds_error=False)

            r = mr(feedback.feedback.base_position.pose.position.y)
            c = mc(feedback.feedback.base_position.pose.position.x)
            rospy.loginfo((int(c),int(r)))

            # plt.imshow(pic_mat,cmap='gray',origin='lower')
            # plt.draw()
            # plt.pause(0.001)

            frontier_pnts = findFrontier(pic_mat)

            mw = interp1d([0,width],[ox,ox+width*res],bounds_error=False)
            mh = interp1d([0,height],[oy,oy+height*res],bounds_error=False)

            if len(frontier_pnts) > 0:
                idx = find_closest([int(c),int(r)],frontier_pnts)
                nextpnt = frontier_pnts[idx]
                rospy.loginfo([nextpnt,prevpnt])
                if prevpnt == 0:
                    checkpnt+=1
                    rospy.loginfo('checkpoint increased')
                    rospy.loginfo(checkpnt)
                else:
                    checkpnt = 0

                if np.sum((np.asarray(nextpnt) - np.asarray(prevpnt))**2) < 200:
                    frontier_pnts.remove(nextpnt)
                    if len(frontier_pnts) > 0:
                        idx = find_closest([int(c),int(r)],frontier_pnts)
                        nextpnt = frontier_pnts[idx]
                        rospy.loginfo('Sent second closest')
                    else:
                        send = False
                        rospy.loginfo('Mapping Done')
                if send == True:
                    prevpnt = nextpnt
                    nextpnt = [mw(nextpnt[0]),mh(nextpnt[1])]
                    rospy.loginfo(nextpnt)
                    sendGoal(nextpnt)
            else:
                rospy.loginfo('Mapping Done')

def sendGoal(nextpnt):
    q = quaternion_from_euler(0,0,0,'sxyz')
    newPose.header.stamp = rospy.Time.now()
    newPose.header.frame_id = "map"
    newPose.pose.position.x = nextpnt[0]
    newPose.pose.position.y = nextpnt[1]
    newPose.pose.orientation.x = q[0]
    newPose.pose.orientation.y = q[1]
    newPose.pose.orientation.z = q[2]
    newPose.pose.orientation.w = q[3]
    pub.publish(newPose)


def find_closest(node,nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

def findFrontier(mat):
    frontier_pnts = []
    # frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    dx = [0,-1,-1,-1,0,1,1,1]
    dy = [1,1,0,-1,-1,-1,0,1]

    frontier_mat = np.array(mat).astype(np.uint8)
    frontier_mat = cv2.Canny(frontier_mat,100,200)
    # plt.imshow(frontier_mat,cmap='gray',origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    free_pnts = np.asarray(np.where(frontier_mat==255)).T.tolist()
    frontier_mat = np.zeros(np.shape(mat),dtype=np.uint8).tolist()
    row, col = np.shape(mat)
    for j in range(len(free_pnts)):
        r,c = free_pnts[j]
        if mat[r][c] == 100:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 50:
                        frontier_mat[r][c] = 255
                        break
        elif mat[r][c] == 50:
            for i in range(8):
                r1 = r + dx[i]
                c1 = c + dy[i]

                if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                    if mat[r1][c1] == 100:
                        frontier_mat[r][c] = 255
                        break
    # plt.imshow(frontier_mat,cmap='gray',origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    frontmat = np.array(frontier_mat).astype(np.uint8)
    kernel = np.ones((5,5), np.uint8)*255
    frontmat = cv2.dilate(frontmat,kernel,iterations=3)
    contour = cv2.findContours(frontmat.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(contour) > 0:
        res = np.array(mat).astype(np.uint8)
        for i in range(len(contour)):
            c = contour[i]
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if radius > 25:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                frontier_pnts.append(center)
                res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),75,2)

        # plt.imshow(res,cmap='gray',origin='lower')
        # plt.draw()
        # plt.pause(0.001)

    return frontier_pnts


if __name__ == "__main__":
    rospy.init_node('gridcalc') #make node
    rospy.sleep(1)
    sendGoal([0.5,0.5])
    rospy.Subscriber('/map', OccupancyGrid, mapCb,queue_size=1)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costCb,queue_size=1)
    rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, costupCb,queue_size=1)
    rospy.Subscriber('move_base/result', MoveBaseActionResult, sendNavCb)
    rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, feedbackCb)
    # plt.figure()
    # plt.show()
    rospy.spin()
