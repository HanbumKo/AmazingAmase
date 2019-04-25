import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi, voronoi_plot_2d
import math
import sys

from scipy.spatial import ConvexHull
import time

class StraightSearch():

    def __init__(self, pointlist, number_keepinzone, number_recoveryzone, numberofdroneeachrecoveryzone):
        self.points = pointlist
        self.number_keepinzone = number_keepinzone
        self.number_recoveryzone = number_recoveryzone
        self.number_drone_each_recoveryzone = numberofdroneeachrecoveryzone
        self.searchcoord = []
        self.searchroute = []
        width = 64373.76
        height = 64373.76
        self.keepinzonewidth = width
        self.keepinzoneheight = height
        # width = boundary.get_Width()
        # height = boundary.get_Height()


        self.__width = width / 100000
        self.__height = height / 100000

    '''
    findtouchingpointkeepinzone에서 호출하는 함수, 2개의 포인트를 연결한 라인과 keepinzone과 교차하는 coordinate를 return
    '''
    def findintersectionpoint(self, src, dst):
        # print("findintersectionpoint)")
        # print(src,dst)

        sx = self.points[2][0]
        lx = self.points[0][0]
        sy = self.points[0][1]
        ly = self.points[2][1]
        # print("src,dst ",src,dst)
        a = (src[1] - dst[1]) / (src[0] - dst[0])
        b = src[1] - (a * src[0])
        # print("a,b\n",a,b)
        # print("polygon idx, i\n",polygon[idx][0], polygon[i][0])

        # 오른쪽
        if src[0] < dst[0]:
            # print("RIGHT")
            # 위
            if src[1] < dst[1]:
                # print("UP")
                standard = (self.points[1][1] - src[1]) / (self.points[1][0] - src[0])
                if a > standard:
                    # print("LY")
                    x = (ly - b) / a
                    x = round(x, 4)
                    y = ly
                else:
                    # print("LX")
                    y = a * lx + b
                    y = round(y, 4)
                    x = lx
            # 아래
            elif src[1] > dst[1]:
                # print("DOWN")
                standard = (self.points[0][1] - src[1]) / (self.points[0][0] - src[0])
                if a > standard:
                    # print("LX")
                    y = a * lx + b
                    y = round(y, 4)
                    x = lx
                else:
                    # print("SY")
                    x = (sy - b) / a
                    x = round(x, 4)
                    y = sy
        # 왼쪽
        elif src[0] > dst[0]:
            # print("LEFT")
            # 위
            if src[1] < dst[1]:
                # print("UP")
                standard = (self.points[2][1] - src[1]) / (self.points[2][0] - src[0])
                if a > standard:
                    # print("SX")
                    y = a * sx + b
                    y = round(y, 4)
                    x = sx
                else:
                    # print("LY")
                    x = (ly - b) / a
                    x = round(x, 4)
                    y = ly
            # 아래
            elif src[1] > dst[1]:
                # print("DOWN")
                standard = (self.points[3][1] - src[1]) / (self.points[3][0] - src[0])
                if a > standard:
                    # print("SY")
                    x = (sy - b) / a
                    x = round(x, 4)
                    y = sy
                else:
                    # print("SX")
                    y = a * sx + b
                    y = round(y, 4)
                    x = sx
        return [x, y]

    def findtouchingpointkeepinzone(self, polygon, region, in_keepinzone_points_coord, in_keepinzone_points_list):
        # print("findtouchingpointkeepinzone")
        new_in_keepinzone_points_coord = in_keepinzone_points_coord
        intersectionpoint = []
        # print(polygon)

        for i in range(len(region)):
            # keepinzone 안에 해당되는 점이면 양쪽 옆의 점들을 보고 keepinzone 안에 없을 시 keepinzone과 만나는지점 저장
            if region[i] in in_keepinzone_points_list:
                if not region[(i-1)%len(region)] in in_keepinzone_points_list:
                    intersectionpoint = self.findintersectionpoint(polygon[i], polygon[(i-1)%len(region)])
                    new_in_keepinzone_points_coord.append(intersectionpoint)
                if not region[(i+1)%len(region)] in in_keepinzone_points_list:
                    intersectionpoint = self.findintersectionpoint(polygon[i], polygon[(i+1)%len(region)])
                    new_in_keepinzone_points_coord.append(intersectionpoint)
        #         print("intersectionpoint\n",intersectionpoint)
        #         print(region[i])
        #print("new_in_keepinzone_points_coordtttt\n",new_in_keepinzone_points_coord)
        return new_in_keepinzone_points_coord

    '''
    keepinzone의 가까운 꼭지점을 삽입
    '''
    def insertkeepinzonevertex(self, in_keepinzone_points_coord, polygon_count):
        #print("insertkeepinzonevertex\n",type(in_keepinzone_points_coord))
        min_d = [0, 0, 0, 0]
        dis = [[0 for _ in range(self.number_recoveryzone)] for _ in range(self.number_keepinzone)]
        for i in range(self.number_keepinzone):
            for j in range(self.number_recoveryzone):
                dis[i][j] = self.caldistancebetweentwopoint(self.points[i],
                                                            self.points[self.number_keepinzone + j])

        #print("dis\n",np.array(dis))
        for i in range(self.number_keepinzone):
            min_d[i] = dis[i].index(min(dis[i]))
        for i in range(len(min_d)):
            if polygon_count == min_d[i]:
                in_keepinzone_points_coord.append(self.points[i].tolist())
        return in_keepinzone_points_coord

    def caldistancebetweentwopoint(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        d = math.sqrt((dx * dx) + (dy * dy))
        return d

    def returnangle(self, points):
        ag = np.zeros([3])
        for i in range(len(points)):
            d1 = points[(i+1)%3] - points[(i%3)]
            d2 = points[(i-1)%3] - points[(i%3)]
            cosine_angle = np.dot(d1, d2) / (np.linalg.norm(d1) * np.linalg.norm(d2))
            angle = np.arccos(cosine_angle)
            ag[i] = np.degrees(angle)
        #print(ag)
        if np.min(ag) < 45:
            minidx = np.where(ag == np.min(ag))
            ag[minidx[0]] = 180
            minidx2 = np.where(ag == np.min(ag))
            minidx = minidx[0]
            minidx2 = minidx2[0]

            dx = points[minidx][0][0] - points[minidx2][0][0]
            dy = points[minidx][0][1] - points[minidx2][0][1]
            if dx !=0:
                a = dy / dx
            else:
                a = float('Inf')
            # t=a
            a = math.atan2(dy,dx)*180/math.pi
            # print(t)
            # print(minidx, minidx2)
            # if a< -360:
            #     print(points[minidx][0][0], points[minidx2][0][0])
            #     print(points[minidx][0][1], points[minidx2][0][1])
            #     print("!!!!!")
            return 1 , a
        else:
            return 0, None

    def caldistance(self, points):
        distance = [0,0,0]
        for i in range(len(points)):
            dx = points[(i + 1) % 3][0] - points[i % 3][0]
            dy = points[(i + 1) % 3][1] - points[i % 3][1]

            d = math.sqrt((dx * dx) + (dy * dy))
            distance[i] = d
        return distance

    def areasize(self, points):
        d = self.caldistance(points)

        s = (d[0] + d[1] + d[2]) / 2
        r1 = (s * (s - d[0]) * (s - d[1]) * (s - d[2])) ** 0.5

        return r1

    def infotriangles(self, adjusted_polygon_points, trilist):

        coordlist = []
        degreelist = []
        arealist = []
        for i in range(len(trilist.simplices)):
            x = 0
            y = 0
            temp = []
            for j in range(3):
                temp.append(adjusted_polygon_points[trilist.simplices[i][j]])
                x += adjusted_polygon_points[trilist.simplices[i][j]][0]
                y += adjusted_polygon_points[trilist.simplices[i][j]][1]
            x /= 3
            y /= 3
            coordlist.append([x,y])
            arealist.append(self.areasize(temp))

        return coordlist, arealist

    def calcdistancecenterandrecovery(self, coordlist, recoveryzone):
        distance = []
        distance_idx = [0 for _ in range(len(coordlist))]
        for t in range(len(coordlist)):
            distance.append(self.caldistancebetweentwopoint(coordlist[t], recoveryzone))

        sorteddistance = distance[:]
        sorteddistance.sort()

        for i in range(len(sorteddistance)):
            for j in range(len(coordlist)):
                if sorteddistance[i] == distance[j]:
                    distance_idx[i] = j
                    continue

        return distance_idx

    def sortaraesize(self, arealist):
        sortedarealist_idx = [0 for _ in range(len(arealist))]
        sortedarealist = arealist[:]
        sortedarealist.sort()
        for i in range(len(sortedarealist)):
            for j in range(len(arealist)):
                if sortedarealist[i] == arealist[j]:
                    sortedarealist_idx[i] = j
        return sortedarealist_idx


    def findnearestlist(self, coordlist, startidx):

        distance = [[0 for _ in range(len(coordlist))] for _ in range(len(coordlist))]
        greedyidx = [0 for _ in range(len(coordlist))]
        for i in range(len(coordlist)):
            for j in range(len(coordlist)):
                distance[i][j] = self.caldistancebetweentwopoint(coordlist[i], coordlist[j])
                if i == j :
                    distance[i][j] = sys.maxsize
        #print("distance\n",distance)

        check = [False for _ in range(len(coordlist))]
        check[startidx] = True

        for i in range(len(coordlist)):
            greedyidx[i] = startidx
            temp = np.array(distance[startidx])
            #print("temp\n",temp)
            for j in range(len(temp)):
                startidx = np.argmin(temp)
                if not check[startidx]:
                    check[startidx] = True
                    break
                else:
                    temp[startidx] = sys.maxsize
        #print(distance)
        return distance, greedyidx

    def calcxydistance(self,p1,p2):
        pass

    def decideway(self):
        total_dx = 0
        total_dy = 0
        recoveryzone = self.points[4:]
        print(self.points)
        print(recoveryzone)
        for i in range(len(recoveryzone)):
            dx = recoveryzone[i][0] - recoveryzone[(i + 1) % len(recoveryzone)][0]
            dy = recoveryzone[i][1] - recoveryzone[(i + 1) % len(recoveryzone)][1]
            total_dx += math.sqrt(dx * dx)
            total_dy += math.sqrt(dy * dy)
        total_dx /= 3
        total_dy /= 3
        decidedway = 0
        if total_dx >  total_dy:
            decidedway = 0
        else:
            decidedway = 1
        return decidedway

    def straightalgo(self):

        sx = self.points[2][0]
        lx = self.points[0][0]
        sy = self.points[0][1]
        ly = self.points[2][1]

        way = self.decideway()
        '''
        way = 0 => longtitude 비슷
        way = 1 => latitude 비슷
        '''
        # print(way)
        boundry_width_term = self.__width / self.number_recoveryzone
        boundry_height_term = self.__height / self.number_recoveryzone
        # print(boundry_width_term, boundry_height_term)
        # print(boundry_width_term*3)
        # print(self.__height)
        drone_width_term = boundry_width_term / self.number_drone_each_recoveryzone
        drone_height_term = boundry_height_term / self.number_drone_each_recoveryzone
        keep_top_left = self.points[2]
        keep_bottom_right = self.points[0]
        '''
        Visualization
        '''
        fig, [[ax1, ax2],[ax3, ax4]]= plt.subplots(nrows=2, ncols=2)

        ax1.set_xlim([self.points[2][0] - 0.05, self.points[1][0] + 0.05])
        ax1.set_ylim([self.points[0][1] - 0.05, self.points[1][1] + 0.05])
        ax2.set_xlim([self.points[2][0] - 0.05, self.points[1][0] + 0.05])
        ax2.set_ylim([self.points[0][1] - 0.05, self.points[1][1] + 0.05])
        ax3.set_xlim([self.points[2][0] - 0.05, self.points[1][0] + 0.05])
        ax3.set_ylim([self.points[0][1] - 0.05, self.points[1][1] + 0.05])
        ax1.plot(self.points[:, 0], self.points[:, 1], 'o')
        for i in range(len(self.points)):
            ax1.text(self.points[i][0],self.points[i][1],'{}'.format(i))

        drone_start_position = [[[] for _ in range(self.number_drone_each_recoveryzone)] for _ in range(self.number_recoveryzone)]
        box_of_list = [[] for _ in range(self.number_recoveryzone)]
        print(drone_start_position)
        test = []
        # latitude차이가 적어서 longtitude로 나눔
        interval_box = self.__height / self.number_recoveryzone
        drone_interval = interval_box / (6 * self.number_drone_each_recoveryzone)
        print(interval_box)

        if way:

            # long위에

            for i in range(self.number_recoveryzone):
                box_of_list[i].append([keep_top_left[0], keep_top_left[1]-interval_box*i])
                box_of_list[i].append([keep_bottom_right[0], keep_top_left[1] - interval_box * i])
                test.append(np.array(np.array(box_of_list[i][0])+np.array(box_of_list[i][1])+np.array([keep_top_left[0], keep_top_left[1]-interval_box*(i+1)])+np.array([keep_bottom_right[0], keep_top_left[1] - interval_box * (i+1)])) / 4)

                #box_of_list.append([keep_top_left[0], keep_top_left[1] - interval_box_list * (i+1)])
                for j in range(self.number_drone_each_recoveryzone):
                    drone_start_position[i][j].append([box_of_list[i][0][0] + drone_interval, box_of_list[i][0][1] - (drone_interval*6*j)- drone_interval])
                    drone_start_position[i][j].append([box_of_list[i][1][0] - drone_interval, box_of_list[i][1][1] - (drone_interval*6*j)- drone_interval])
                    drone_start_position[i][j].append([box_of_list[i][1][0] - drone_interval, box_of_list[i][1][1] - (drone_interval*6*j)- drone_interval*5])
                    drone_start_position[i][j].append([box_of_list[i][0][0] + drone_interval, box_of_list[i][0][1] - (drone_interval*6*j)- drone_interval*5])

        else:
            for i in range(self.number_recoveryzone):
                box_of_list[i].append([keep_top_left[0] + interval_box * i, keep_top_left[1]])
                box_of_list[i].append([keep_top_left[0] + interval_box * i, keep_bottom_right[1]])
                test.append(np.array(np.array(box_of_list[i][0])+np.array(box_of_list[i][1])+np.array([keep_top_left[0] + interval_box * (i+1), keep_top_left[1]])+np.array([keep_top_left[0] + interval_box * (i+1), keep_bottom_right[1]])) / 4)

                for j in range(self.number_drone_each_recoveryzone):
                    # drone_start_position[i][j].append([box_of_list[i][0][0] + drone_interval, box_of_list[i][0][1] - (drone_interval*6*j)- drone_interval])
                    # drone_start_position[i][j].append([box_of_list[i][1][0] - drone_interval, box_of_list[i][1][1] - (drone_interval*6*j)- drone_interval])
                    # drone_start_position[i][j].append([box_of_list[i][1][0] - drone_interval, box_of_list[i][1][1] - (drone_interval*6*j)- drone_interval*5])
                    # drone_start_position[i][j].append([box_of_list[i][0][0] + drone_interval, box_of_list[i][0][1] - (drone_interval*6*j)- drone_interval*5])

                    drone_start_position[i][j].append([box_of_list[i][0][0] + (drone_interval * 6 * j) + drone_interval, box_of_list[i][0][1] - drone_interval])
                    drone_start_position[i][j].append([box_of_list[i][1][0] + (drone_interval * 6 * j) + drone_interval, box_of_list[i][1][1] + drone_interval])
                    drone_start_position[i][j].append([box_of_list[i][1][0] + (drone_interval * 6 * j) + drone_interval*5, box_of_list[i][1][1] + drone_interval])
                    drone_start_position[i][j].append([box_of_list[i][0][0] + (drone_interval * 6 * j) + drone_interval*5, box_of_list[i][0][1] - drone_interval])




        test = np.array(test)
        ax1.plot(test[:, 0], test[:, 1], 'v')

        for i in range(len(test)):
            ax1.text(test[i][0],test[i][1],'{}'.format(i))

        box_of_list = np.array(box_of_list)
        drone_start_position = np.array(drone_start_position)
        print("box_of_list\n", box_of_list)
        print("drone_start_position\n",drone_start_position)
        for i in range(len(box_of_list)):
            ax2.plot(box_of_list[i][:, 0], box_of_list[i][:, 1], 'v')

        ax3.plot(self.points[:, 0], self.points[:, 1], 'o')

        for i in range(len(box_of_list)):
            ax3.plot(box_of_list[i][:, 0], box_of_list[i][:, 1], 'ko')
            for j in range(len(drone_start_position[i])):
                ax3.plot(drone_start_position[i][j][:, 0], drone_start_position[i][j][:, 1], 'v')
        # print(box_of_list)
        # print(drone_start_position)


        plt.show()
        return drone_start_position



if __name__ == '__main__':
    nkeepinzone = 4
    nrecoveryzone = 3
    numberofdroneeachrecoveryzone = 3
    pointlist = [[] for _ in range((nkeepinzone + nrecoveryzone))]
    # [[40.12915016547195, -121.38244270915412], [40.12915016547195, -120.73870509196662],
    #  [39.48541254828445, -120.73870509196662], [39.48541254828445, -121.38244270915412],
    #  [39.961441084517446, -121.36296219605264], [39.899205872511864, -120.80022386724143],
    #  [39.642815077015385, -120.97016923347738]]

    # # LEFT DOWN
    # pointlist[0] = [40.12915017 - 0.04, -121.4366 - 0.03]
    # # LEFT UP
    # pointlist[1] = [40.12915017 - 0.04, -120.6866 + 0.03]
    # # RIGHT UP
    # pointlist[2] = [39.5177 + 0.04, -120.6866 + 0.03]
    # # RIHGT DOWN
    # pointlist[3] = [39.5177 + 0.04, -121.4366 - 0.03]
    #
    # LEFT DOWN
    pointlist[0] = [40.12915016547195, -121.38244270915412]
    # LEFT UP
    pointlist[1] = [40.12915016547195, -120.73870509196662]
    # RIGHT UP
    pointlist[2] = [39.48541254828445, -120.73870509196662]
    # RIHGT DOWN
    pointlist[3] = [39.48541254828445, -121.38244270915412]

    # pointlist[0][0] -= 0.04
    # pointlist[1][0] -= 0.04
    # pointlist[2][0] += 0.04
    # pointlist[3][0] += 0.04
    # pointlist[0][1] -= 0.04
    # pointlist[1][1] += 0.04
    # pointlist[2][1] += 0.04
    # pointlist[3][1] -= 0.04

    # pointlist[4] = [39.9258, -121.2517]
    # pointlist[5] = [39.9919, -120.8328]
    # pointlist[6] = [39.5894, -121.0448]
    # pointlist[7] = [39.7181, -121.254]
    # pointlist[8] = [39.8012, -121.1]
    # pointlist[9] = [39.8094, -120.828]
    # [[40.0958 - 121.4353]
    #  [40.0958 - 120.6826]
    #  [39.5175 - 120.6826]
    #  [39.5175 - 121.4353]
    #  [39.83631523 - 121.40076423]
    #  [39.77640412 - 120.72580182]
    #  [39.79598009 - 121.05816625]]
    # 2_A
    pointlist[4] = [39.83631523, -121.40076423]
    pointlist[5] = [39.77640412, -120.72580182]
    pointlist[6] = [39.79598009, -121.05816625]



    # 4 competition
    #  [40.06657647452135, -121.39191396055425],
    #  [39.56880445204028, -120.85666124971846],
    #  [39.69132807055441, -121.09272006829518]]

    # pointlist[4] = [40.0409, -121.0774]
    # pointlist[5] = [39.9228, -120.9898]
    # pointlist[6] = [39.8047, -121.0645]
    # pointlist[7] = [39.5597, -121.0171]

    pointlist = np.array(pointlist)
    '''
    startway
    0 = nearest
    1 = farthest
    2 = smallest
    3 = largest
    '''
    straight = StraightSearch(pointlist, nkeepinzone, nrecoveryzone, numberofdroneeachrecoveryzone)
    waypoints = straight.straightalgo()
    print("waypoint\n",waypoints)
