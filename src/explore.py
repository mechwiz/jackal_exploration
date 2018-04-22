#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.path as mpltPath

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
# from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

cost_update = OccupancyGridUpdate()
result = MoveBaseActionResult()
newPose = PoseStamped()
q = Quaternion()


class jackal_explore:
    def __init__(self):
        self.costmap = OccupancyGrid()
        self.flagm = 0
        self.flagg = 0
        self.init = 0
        self.newcost = []
        self.grid = OccupancyGrid()
        self.feedback = MoveBaseActionFeedback()
        self.prevpnt = -1
        self.nextpnt = 0
        self.sample = []
        self. stuckpnt = []
        self.isstuck = False
        self.offsetcnt = 0
        self.firstgoal = 0
        self.checkpnt = 0
        self.pntlist = []
        self.poly = []
        self.polynum = 0
        self.polydone = False
        self.polypath = []
        self.count_id = 1
        self.send = True
        self.currentpos =[]

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb,queue_size=1)
        self.global_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costCb,queue_size=1)
        self.globalup_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costupCb,queue_size=1)
        self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
        self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)
        self.poly_sub = rospy.Subscriber('/clicked_point',PointStamped,self.polyCb,queue_size=1)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.line_pub = rospy.Publisher('visualization_marker',Marker,queue_size = 10)

    def polyCb(self,data):

        if self.polydone == True:
            for i in range(self.count_id-1):
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "points_and_lines"
                marker.id = i
                if i == 0:
                    marker.type = marker.LINE_STRIP
                else:
                    marker.type = marker.SPHERE
                marker.action = marker.DELETE
                self.line_pub.publish(marker)

            self.polydone = False
            self.poly = []
            self.polynum = 0
            self.polypath = []
            self.polynum = 0
            self.count_id = 1


        x = data.point.x
        y = data.point.y
        rospy.loginfo([x,y])

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points_and_lines"
        marker.id = self.count_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.g = 1.0
        marker.color.a = 1.0
        self.line_pub.publish(marker)

        self.count_id += 1
        self.polynum += 1


        if self.polynum > 3:
            closepnt = self.poly[0]
            if calc_distance([x,y],closepnt) < 1:
                rospy.loginfo('Boundary done')
                self.polypath = mpltPath.Path(np.array(self.poly))
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "points_and_lines"
                marker.id = self.count_id - 1
                marker.type = marker.SPHERE
                marker.action = marker.DELETE

                self.line_pub.publish(marker)
                self.polydone = True
                self.send = True
                self.sendGoal(self.currentpos)


        if self.polydone == False:
            self.poly.append([x,y])

        if len(self.poly) > 1:
            newMarker = Marker()
            newMarker.header.frame_id = "/map"
            newMarker.header.stamp = rospy.Time.now()
            newMarker.ns = "points_and_lines"
            newMarker.id = 0
            newMarker.type = newMarker.LINE_STRIP
            newMarker.action = newMarker.ADD
            newMarker.pose.orientation.w = 1



            newMarker.scale.x = 0.05
            newMarker.color.b = 1.0
            newMarker.color.a = 1.0

            for p in self.poly:
                pnt = Point()
                pnt.x = p[0]
                pnt.y = p[1]
                newMarker.points.append(pnt)

            if self.polydone == True:
                pnt = Point()
                pnt.x = self.poly[0][0]
                pnt.y = self.poly[0][1]
                newMarker.points.append(pnt)
                newMarker.color.b = 0
                newMarker.color.r = 1.0

            self.line_pub.publish(newMarker)

    def costupCb(self,data):
        self.flagg = 1
        cost_update = data

        if self.init == 1 and self.flagm == 0:
            gpnts = cost_update.data
            gwidth = cost_update.width
            gheight = cost_update.height
            gx = cost_update.x
            gy = cost_update.y
            cnt = 0
            for x in range(gheight):
                for y in range(gwidth):
                    g = gpnts[cnt]
                    self.newcost[x+gy][y+gx] = g
                    cnt += 1

    def costCb(self,data):
        self.newcost = []
        self.flagm = 1
        self.costmap = data

        width = self.costmap.info.width
        height = self.costmap.info.height
        pnts = self.costmap.data
        cnt = 0

        for x in range(height):
            newcost_x = []
            for y in range(width):
                newcost_x.append(pnts[cnt])
                cnt+=1
            self.newcost.append(newcost_x)

    def mapCb(self,data):
        self.grid = data

        if self.flagm == 1:

            self.init = 1
            rospy.loginfo('received1')
            res = self.grid.info.resolution
            width = self.grid.info.width
            height = self.grid.info.height
            pnts = self.grid.data
            gpnts = self.costmap.data
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
            self.flagm = 0

            # plt.imshow(pic_mat,cmap='gray',origin='lower')
            # plt.draw()
            # plt.pause(0.001)

    def feedbackCb(self,data):
        offsetx = [1,-1,1,-1]
        offsety = [1,-1,-1,1]

        self.feedback = data
        x = self.feedback.feedback.base_position.pose.position.x
        y = self.feedback.feedback.base_position.pose.position.y
        w = self.feedback.feedback.base_position.pose.orientation.w
        self.sample.append([x,y,w])
        self.currentpos = [x,y]

        if self.firstgoal == 0:
            self.sendGoal([x+0.5,y+0.5])
            self.firstgoal = 1

        if len(self.sample) > 15:
            sa = np.array(self.sample)
            mx = np.average(sa[:,0])
            my = np.average(sa[:,1])
            mw = np.average(sa[:,2]) - w
            # rospy.loginfo(calc_distance([x,y],[mx,my]))

            if calc_distance([x,y],[mx,my]) < 0.015 and abs(mw)<0.01:
                rospy.loginfo('Stuck. Resetting Goal...')
                self.isstuck = True
                xp, yp = x,y

                if len(self.stuckpnt) > 0:
                    sx = self.stuckpnt[0]
                    sy = self.stuckpnt[1]
                    if calc_distance([x,y],[sx,sy]) < 0.1:
                        if self.checkpnt < 4 and self.prevpnt != 0:
                            self.prevpnt = 0
                            rospy.loginfo("Changing Point")
                            x = x + 0.5*offsetx[self.offsetcnt]
                            y = y + 0.5*offsety[self.offsetcnt]
                            self.offsetcnt += 1
                            if self.offsetcnt > 3:
                                self.offsetcnt = 0
                        # else:
                        #     rospy.loginfo('Adding taboo point')
                        #     self.pntlist.append(self.nextpnt)
                    elif self.checkpnt > 0 and self.prevpnt != 0:
                        self.prevpnt = 1

                    if self.checkpnt == 4:
                        rospy.loginfo('Adding taboo point')
                        self.pntlist.append(self.nextpnt)
                        self.checkpnt = 0
                self.stuckpnt = [xp,yp]
                self.sendGoal([x,y])
            self.sample = []

    def sendNavCb(self,data):
        self.sample = []
        result = data
        frontier_mat = []
        if result.status.text == "Goal reached.":
            if self.flagg == 1 and self.init == 1:
                self.flagg = 0
                rospy.loginfo('received2')
                res = self.grid.info.resolution
                width = self.grid.info.width
                height = self.grid.info.height
                pnts = self.grid.data
                gpnts = np.array(self.newcost).flatten().tolist()
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

                ox = self.grid.info.origin.position.x
                oy = self.grid.info.origin.position.y
                mc = interp1d([ox,ox+width*res],[0,width],bounds_error=False)
                mr = interp1d([oy,oy+height*res],[0,height],bounds_error=False)

                r = mr(self.feedback.feedback.base_position.pose.position.y)
                c = mc(self.feedback.feedback.base_position.pose.position.x)
                rospy.loginfo((int(c),int(r)))

                # plt.imshow(pic_mat,cmap='gray',origin='lower')
                # plt.draw()
                # plt.pause(0.001)

                mw = interp1d([0,width],[ox,ox+width*res],bounds_error=False)
                mh = interp1d([0,height],[oy,oy+height*res],bounds_error=False)

                frontier_pnts = findFrontier(pic_mat)

                if self.polydone == True and len(frontier_pnts) > 0:
                    frontier_filter = []
                    rospy.loginfo('filter occuring')
                    for j in range(len(frontier_pnts)):
                        fx = mw(frontier_pnts[j][0])
                        fy = mh(frontier_pnts[j][1])
                        if self.polypath.contains_point([fx,fy]) == True:
                            frontier_filter.append(frontier_pnts[j])

                    frontier_pnts = frontier_filter

                if len(frontier_pnts) > 0:
                    idx = find_closest([int(c),int(r)],frontier_pnts)
                    nextpnt = frontier_pnts[idx]
                    pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]

                    if self.pntlist:
                        for i in range(len(self.pntlist)):
                            if calc_distance(pnt1,self.pntlist[i]) < 5:
                                frontier_pnts.remove(nextpnt)
                                if len(frontier_pnts) > 0:
                                    idx = find_closest([int(c),int(r)],frontier_pnts)
                                    nextpnt = frontier_pnts[idx]
                                    pnt1 = [mw(nextpnt[0]),mh(nextpnt[1])]
                                    rospy.loginfo('sending number '+str(i+2) + ' in list')
                                else:
                                    rospy.loginfo('Mapping Done')
                                    self.send = False


                    # rospy.loginfo([nextpnt,self.prevpnt])
                    # if self.prevpnt == 0:
                    #     self.checkpnt+=1
                    #     rospy.loginfo('checkpoint increased')
                    #     rospy.loginfo(self.checkpnt)
                    # elif self.prevpnt == 1:
                    #     rospy.loginfo('wait to change point')
                    # else:
                    #     self.checkpnt = 0

                    if self.isstuck == False and self.prevpnt != -1:
                        if calc_distance(nextpnt,self.prevpnt) < 5:
                            rospy.loginfo('repeated point')
                            frontier_pnts.remove(nextpnt)
                            if len(frontier_pnts) > 0:
                                idx = find_closest([int(c),int(r)],frontier_pnts)
                                nextpnt = frontier_pnts[idx]
                                rospy.loginfo('Sent second closest')
                            else:
                                self.send = False
                                rospy.loginfo('Mapping Done')

                    if self.send == True:
                        rospy.loginfo([nextpnt,self.prevpnt])
                        if self.prevpnt == 0:
                            self.checkpnt+=1
                            rospy.loginfo('checkpoint increased')
                            rospy.loginfo(self.checkpnt)
                        elif self.prevpnt == 1:
                            rospy.loginfo('wait to change point')
                        else:
                            self.checkpnt = 0

                        self.prevpnt = nextpnt
                        self.nextpnt = [mw(nextpnt[0]),mh(nextpnt[1])]
                        rospy.loginfo(nextpnt)
                        self.sendGoal(self.nextpnt)
                        self.isstuck = False
                else:
                    rospy.loginfo('Mapping Done')

    def sendGoal(self,nextpnt):
        q = quaternion_from_euler(0,0,0,'sxyz')
        newPose.header.stamp = rospy.Time.now()
        newPose.header.frame_id = "map"
        newPose.pose.position.x = nextpnt[0]
        newPose.pose.position.y = nextpnt[1]
        newPose.pose.orientation.x = q[0]
        newPose.pose.orientation.y = q[1]
        newPose.pose.orientation.z = q[2]
        newPose.pose.orientation.w = q[3]
        self.pub.publish(newPose)

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

        # if self.polydone == True and len(frontier_pnts) > 0:
        #     frontier_filter = []
        #     rospy.loginfo('filter occuring')
        #     for j in range(len(frontier_pnts)):
        #         if self.polypath.contains_point(frontier_pnts[j]) == True:
        #             frontier_filter.append(frontier_pnts[j])

        #     frontier_pnts = frontier_filter
        # plt.imshow(res,cmap='gray',origin='lower')
        # plt.draw()
        # plt.pause(0.001)

    return frontier_pnts

def calc_distance(pnt1,pnt2):
    x1,y1 = pnt1
    x2,y2 = pnt2
    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def find_closest(node,nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

if __name__ == "__main__":
    rospy.init_node('explore') #make node
    rospy.sleep(1)
    gc = jackal_explore()
    rospy.sleep(1)
    gc.sendGoal([0.5,0.5])
    rospy.sleep(0.5)
    # plt.figure()
    # plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
