import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi
import math
import time
import copy

class drawvoronoi():
    def __init__(self, pointlist, number_keepinzone, number_recoveryzone):
        # get points from keep in zone and recovery zone => point_list
        self.point_list = pointlist
        self.number_keepinzone = number_keepinzone
        self.number_recoveryzone = number_recoveryzone
        print()
        print(self.point_list)
        # self.number_keepinzone = 4
        # self.number_recoveryzone = 5
        # self.point_list = [[] for _ in range((self.number_keepinzone + self.number_recoveryzone))]
        # scenario = "practice3c"
        # # LEFT DOWN
        # self.point_list[0] = [40.12915017 - 0.04, -121.4366 - 0.03]
        # # LEFT UP
        # self.point_list[1] = [40.12915017 - 0.04, -120.6866 + 0.03]
        # # RIGHT UP
        # self.point_list[2] = [39.5177 + 0.04, -120.6866 + 0.03]
        # # RIHGT DOWN
        # self.point_list[3] = [39.5177 + 0.04, -121.4366 - 0.03]
        #
        # self.point_list[4] = [39.9258, -121.2517]
        # self.point_list[5] = [39.9919, -120.8328]
        # self.point_list[6] = [39.5794, -121.0448]
        # self.point_list[7] = [39.7181, -121.254]
        # self.point_list[8] = [39.8012, -121.1]

        self.vor_list_for_loiter = []


    def voronoi_finite_polygons_2d(self, vor, radius=None):
        """
        Reconstruct infinite voronoi regions in a 2D diagram to finite
        regions.
        Parameters
        ----------
        vor : Voronoi
            Input diagram
        radius : float, optional
            Distance to 'points at infinity'.
        Returns
        -------
        regions : list of tuples
            Indices of vertices in each revised Voronoi regions.
        vertices : list of tuples
            Coordinates for revised Voronoi vertices. Same as coordinates
            of input vertices, with 'points at infinity' appended to the
            end.
        """

        if vor.points.shape[1] != 2:
            raise ValueError("Requires 2D input")

        new_regions = []
        new_vertices = vor.vertices.tolist()

        center = vor.points.mean(axis=0)
        if radius is None:
            radius = vor.points.ptp().max()*2

        # Construct a map containing all ridges for a given point
        all_ridges = {}
        for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
            all_ridges.setdefault(p1, []).append((p2, v1, v2))
            all_ridges.setdefault(p2, []).append((p1, v1, v2))

        # Reconstruct infinite regions
        for p1, region in enumerate(vor.point_region):
            vertices = vor.regions[region]

            if all(v >= 0 for v in vertices):
                # finite region
                new_regions.append(vertices)
                continue

            # reconstruct a non-finite region
            ridges = all_ridges[p1]
            new_region = [v for v in vertices if v >= 0]

            for p2, v1, v2 in ridges:
                if v2 < 0:
                    v1, v2 = v2, v1
                if v1 >= 0:
                    # finite ridge: already in the region
                    continue

                # Compute the missing endpoint of an infinite ridge

                t = vor.points[p2] - vor.points[p1] # tangent
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])  # normal

                midpoint = vor.points[[p1, p2]].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                far_point = vor.vertices[v2] + direction * radius

                new_region.append(len(new_vertices))
                new_vertices.append(far_point.tolist())

            # sort region counterclockwise
            vs = np.asarray([new_vertices[v] for v in new_region])
            c = vs.mean(axis=0)
            angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
            new_region = np.array(new_region)[np.argsort(angles)]

            # finish
            new_regions.append(new_region.tolist())

        return new_regions, np.asarray(new_vertices)

    def findintersectionpoint(self, src, dst):
        print("\nfindintersectionpoint\n")

        sx = self.point_list[2][0]
        lx = self.point_list[0][0]
        sy = self.point_list[0][1]
        ly = self.point_list[2][1]
        #print("i\n",idx, i)
        print("src,dst ",src,dst)
        a = (src[1] - dst[1]) / (src[0] - dst[0])
        b = src[1] - (a * src[0])
        #print("a,b\n",a,b)
        #print("polygon idx, i\n",polygon[idx][0], polygon[i][0])

        # 오른쪽
        if src[0] < dst[0]:
            #print("RIGHT")
            # 위
            if src[1] < dst[1]:
                #print("UP")
                standard = (self.point_list[1][1] - src[1])/(self.point_list[1][0] - src[0])
                if a > standard:
                    #print("LY")
                    x = (ly - b)/a
                    x = round(x, 4)
                    y = ly
                else:
                    #print("LX")
                    y = a * lx + b
                    y = round(y, 4)
                    x = lx
            # 아래
            elif src[1] > dst[1]:
                #print("DOWN")
                standard = (self.point_list[0][1] - src[1]) / (self.point_list[0][0] - src[0])
                if a > standard:
                    #print("LX")
                    y = a * lx + b
                    y = round(y, 4)
                    x = lx
                else :
                    #print("SY")
                    x = (sy - b) / a
                    x = round(x, 4)
                    y = sy
        # 왼쪽
        elif src[0] > dst[0]:
            #print("LEFT")
            # 위
            if src[1] < dst[1]:
                #print("UP")
                standard = (self.point_list[2][1] - src[1]) / (self.point_list[2][0] - src[0])
                if a > standard:
                    #print("SX")
                    y = a * sx + b
                    y = round(y, 4)
                    x = sx
                else:
                    #print("LY")
                    x = (ly - b) / a
                    x = round(x, 4)
                    y = ly
            # 아래
            elif src[1] > dst[1]:
                #print("DOWN")
                standard = (self.point_list[3][1] - src[1]) / (self.point_list[3][0] - src[0])
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
        return [x,y]
        #     new_points.append([x,y])
        # new_points.append([round(polygon[idx][0],4),round(polygon[idx][1],4)])
        #
        # return new_points

    '''
    voronoi 알고리즘을 통해 나온 영역들을 keepinzone에 한정하기 위해 좌표를 구함
    keepinzone과 voronoi영역의 교차점을 구함
    '''
    def findtouchingpointkeepinzone(self, polygon, in_keepinzone_vertex_list, vor_list_for_loiter):
        print("\nfindtouchingpointkeepinzone\n")
        new_points = []
        print(polygon)
        print(in_keepinzone_vertex_list)
        in_keepinzone_vertex_list.sort()
        print("sorted",in_keepinzone_vertex_list)
        # idx1 = 0
        # target1 = 0
        # idx2 = 0
        # target2 = 0
        # 모든 면은 무조건 2개아니면 0개가 keepinzone과 만난다.
        print("len(vertex_list)",len(in_keepinzone_vertex_list))
        print("polygon",len(polygon))
        print(len(polygon)-len(in_keepinzone_vertex_list))
        # 모이는 점에 들어가 있는 점이 하나일때 => [2]

        if len(in_keepinzone_vertex_list) == 1:
            idx1 = in_keepinzone_vertex_list[0]
            idx2 = in_keepinzone_vertex_list[0]
            target1 = (idx1 - 1) % len(polygon)
            target2 = (idx2 + 1) % len(polygon)
        else:
            # 모이는 점들이 연속적인 점들로 이루어져 있음 => [2,3,4]
            if (in_keepinzone_vertex_list[-1] - in_keepinzone_vertex_list[0]) == (len(in_keepinzone_vertex_list) - 1) :
                idx1 = in_keepinzone_vertex_list[0]
                target1 = (idx1 - 1) % len(polygon)
                idx2 = in_keepinzone_vertex_list[-1]
                target2 = (idx2 + 1) % len(polygon)
            # 모이는 점들이 불연속 적인 점들로 이루어져 있음 => [1,2,4,0]
            else :
                for i in range(len(in_keepinzone_vertex_list)-1):
                    if in_keepinzone_vertex_list[i] + 1 != in_keepinzone_vertex_list[i+1]:
                        idx1 = in_keepinzone_vertex_list[i]
                        target1 = idx1 + 1
                        idx2 = in_keepinzone_vertex_list[i+1]
                        target2 = idx2 - 1



        print("idx target : ",idx1, target1, idx2, target2)

        # 각각 idx에 대해서 target point를 이용해서 keepinzone에 닿는 좌표를 찾기
        p1 = self.findintersectionpoint(polygon[idx1], polygon[target1])
        p2 = self.findintersectionpoint(polygon[idx2], polygon[target2])
        '''
        VISUALIZATION
        '''
        plt.plot(p1[0], p1[1], 'o')
        plt.plot(p2[0], p2[1], 'o')
        plt.text(p1[0], p1[1], "p1", fontsize=8)
        plt.text(p2[0] + 0.008, p2[1], "p2", fontsize=8)
        print("p1",p1)
        print("p2",p2)
        polygon_area = []
        for i in range(len(in_keepinzone_vertex_list)):
            polygon_area.append(polygon[in_keepinzone_vertex_list][i].tolist())
            vor_list_for_loiter[i] = copy.deepcopy(polygon_area)
        #print("vor_list_for_loiter\n", vor_list_for_loiter)
        polygon_area.append(p1)
        polygon_area.append(p2)
        print("polygon_area\n",polygon_area)
        return polygon_area

    def caldistancebetweentwopoint(self, p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        d = math.sqrt((dx * dx) + (dy * dy))
        return d

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


    '''
    점 세개를 받아서 가장 작은 각도와 그 다음 작은 각도를 확인하고 로이터 방법 저장
    가장 작은 각도와 그 다음 작은 각도를 가지고 있는 인덱스를 저장
    저장한 두 값 리턴
    '''
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

    def findthreetriangle(self, findpointlist):
        # 꼭지점을 하나만 가지고 있는 경우
        findpointlist = findpointlist.tolist()
        threetriangle = [[],[],[]]
        points = [0, 0, 0]
        if len(findpointlist) == 5:
            print("꼭지점 4개")
            t1 = [findpointlist[0], findpointlist[1], findpointlist[4]]
            t2 = [findpointlist[0], findpointlist[2], findpointlist[4]]
            if self.areasize(t1) > self.areasize(t2):
                print("1 > 2")
                print("0,2,3,4 / 0,1,4 / 1,3,4")
                threetriangle[0] = [findpointlist[0], findpointlist[2], findpointlist[3], findpointlist[4]]
                threetriangle[1] = [findpointlist[0], findpointlist[1], findpointlist[4]]
                threetriangle[2] = [findpointlist[1], findpointlist[3], findpointlist[4]]
            else:
                print("1 < 2")
                print("0, 1, 3, 4 / 0, 2 ,4 / 2, 3, 4")
                threetriangle[0] = [findpointlist[0], findpointlist[1], findpointlist[3], findpointlist[4]]
                threetriangle[1] = [findpointlist[0], findpointlist[2], findpointlist[4]]
                threetriangle[2] = [findpointlist[2], findpointlist[3], findpointlist[4]]

        else :
            print("꼭지점 5개")
            if findpointlist[1][0] == findpointlist[3][0] or findpointlist[1][1] == findpointlist[3][1]:
                threetriangle[0] = [findpointlist[0], findpointlist[1], findpointlist[3], findpointlist[5]]
                threetriangle[1] = [findpointlist[0], findpointlist[2], findpointlist[4], findpointlist[5]]
                threetriangle[2] = [findpointlist[3], findpointlist[4], findpointlist[5]]

            elif findpointlist[1][0] == findpointlist[4][0] or findpointlist[1][1] == findpointlist[4][1]:
                threetriangle[0] = [findpointlist[0], findpointlist[1], findpointlist[4], findpointlist[5]]
                threetriangle[1] = [findpointlist[0], findpointlist[2], findpointlist[3], findpointlist[5]]
                threetriangle[2] = [findpointlist[3], findpointlist[4], findpointlist[5]]

        print(len(threetriangle))
        for i in range(len(threetriangle)):
            px = 0
            py = 0
            print(len(threetriangle[i]))
            for j in range(len(threetriangle[i])):
                px += threetriangle[i][j][0]
                py += threetriangle[i][j][1]
                print("j",j)
            px /= (j + 1)
            py /= (j + 1)
            print(px, py)
            points[i] = [px, py]

        print("threetriangle\n",threetriangle)
        return np.array(threetriangle), points


    def voronoialgo(self):
        vor = Voronoi(self.point_list[4:])

        # 만나는 점들을 넣은
        ver = vor.vertices.tolist()
        ver = np.array(ver)
        regions, vertices = voro.voronoi_finite_polygons_2d(vor)

        print("regions\n",regions)
        print("ver\n",len(ver),"\n",ver)

        self.vor_list_for_loiter = [0 for _ in range(len(ver))]
        in_keepinzone_vertex_list = [[] for _ in range(len(self.point_list[4:]))]
        find_touchingpoint_keepinzone_list = [0 for _ in range(len(self.point_list[4:]))]
        vor_list_for_loiter = [0 for _ in range(len(ver))]

        # print("vor_list_for_loiter\n", vor_list_for_loiter)

        sx = self.point_list[2][0]
        lx = self.point_list[0][0]
        sy = self.point_list[0][1]
        ly = self.point_list[2][1]

        # keep
        ver_list = []

        '''
        Voronoi 그리기 & keepinzone과 만나는 지점 찾기
        '''
        t=0
        idx = []
        for region in regions:
            print("-----------",t,"-----------")
            print("region\n", region)

            polygon = vertices[region]
            plt.fill(*zip(*polygon), alpha=0.4)
            for i in range(len(ver)):
                '''
                만나는 점이 keepinzone안에 있는지 확인
                만나는 점에서 keepinzone 밖에서 생길 수 있음
                '''
                if ver[i].round(5)[0] > sx and ver[i].round(5)[0] < lx:
                    if ver[i].round(5)[1] > sy and ver[i].round(5)[1] < ly:
                        idx = np.where(ver[i].round(5) == polygon.round(5))[0]
                        if len(idx) > 0:
                            # 만나는 점이 keepinzone안에 있을 경우
                            in_keepinzone_vertex_list[t].append(idx[0])

            print("in_keepinzone_vertex_list\n", in_keepinzone_vertex_list)

            # 현재 polygon의 좌표들과 keepinzone안에 있는 좌표의 인덱스를 보냄
            find_touchingpoint_keepinzone_list[t] = np.array(self.findtouchingpointkeepinzone(polygon, in_keepinzone_vertex_list[t], vor_list_for_loiter))

            print("find_touchingpoint_keepinzone_list\n",len(find_touchingpoint_keepinzone_list),
                  "\n",find_touchingpoint_keepinzone_list)
            print("vor_list_for_loiter_main\n", len(vor_list_for_loiter),vor_list_for_loiter)

            t += 1
            find_touchingpoint_keepinzone_list = np.array(find_touchingpoint_keepinzone_list)

        '''
       보로노이 알고리즘을 통해 나온 영역을 좌표로 선택하기 위함
       keepinzone의 모서리 부분을 find_touchingpoint_keepinzone_list에 삽입
       '''
        dis = [[0 for _ in range(self.number_recoveryzone)] for _ in range(self.number_keepinzone)]
        for i in range(self.number_keepinzone):
            for j in range(self.number_recoveryzone):
                dis[i][j] = self.caldistancebetweentwopoint(self.point_list[i], self.point_list[self.number_keepinzone + j])
        print("dis\n",dis)
        for i in range(self.number_keepinzone):
            min_d = dis[i].index(min(dis[i]))
            find_touchingpoint_keepinzone_list[min_d] = np.append(find_touchingpoint_keepinzone_list[min_d], [self.point_list[i].tolist()], axis=0)

        '''
       voronoi로 나눈 영역들에 recovery zone 위치 삽입
       '''
        for i in range(self.number_recoveryzone):
            find_touchingpoint_keepinzone_list[i] = np.append(find_touchingpoint_keepinzone_list[i], [self.point_list[i + self.number_keepinzone]], axis=0)
            plt.text(self.point_list[i + self.number_keepinzone][0], self.point_list[i + self.number_keepinzone][1], "{}".format(i),
                     fontsize=8)

        '''
        보로노이 알고리즘을 통해 나온 영역을 들로네 알고리즘으로 나눔
        '''
        trilist = [0 for _ in range(len(find_touchingpoint_keepinzone_list))]
        for i in range(len(find_touchingpoint_keepinzone_list)):
            trilist[i] = Delaunay(find_touchingpoint_keepinzone_list[i])

        '''
            보로노이 알고리즘을 통해 나온 영역을 들로네 알고리즘으로 나눈 삼각형들의 중점을 표시
            '''
        tripoints = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(find_touchingpoint_keepinzone_list))]
        loiterway = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(find_touchingpoint_keepinzone_list))]
        a_degree = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(find_touchingpoint_keepinzone_list))]
        coord = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(find_touchingpoint_keepinzone_list))]
        arealist = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(find_touchingpoint_keepinzone_list))]
        decide = 0
        angle = 0
        t = 0
        for v in range(len(find_touchingpoint_keepinzone_list)):
            for i in range(len(find_touchingpoint_keepinzone_list[v][trilist[v].simplices])):
                temp = []
                x = 0
                y = 0
                for j in range(3):
                    temp.append(find_touchingpoint_keepinzone_list[v][trilist[v].simplices][i][j])
                    x += find_touchingpoint_keepinzone_list[v][trilist[v].simplices][i][j][0]
                    y += find_touchingpoint_keepinzone_list[v][trilist[v].simplices][i][j][1]
                x /= 3
                y /= 3

                # 삼각형의 중점 : x, y
                tripoints[v][i] = find_touchingpoint_keepinzone_list[v][trilist[v].simplices][i]
                # pointss[i] = np.append(findpointlist[v][trilist[v].simplices][i], [[x, y]], axis=0)

                # trilists[v][i] = Delaunay(pointss[i])

                # 로이터 방식과 기울기
                decide, angle = self.returnangle(tripoints[v][i])
                if angle != None and abs(angle) == 90:
                    angle = 0
                if angle != None and abs(angle) == 180:
                    angle = 90
                if angle != None:
                    angle = round(angle, 3)
                # angle = round(angle,5)
                area = round(self.areasize(temp), 4)
                loiterway[v][i] = decide
                a_degree[v][i] = angle
                coord[v][i] = [x, y]
                arealist[v][i] = area
                '''
                Visualization 
                '''
                plt.plot(x, y, 'v')
                plt.text(x, y, 'zonenubmer {}\ntrinumber {}\nangle {}\narea {}'.format(v, i, angle, area), fontsize=6)
            t += 0.005

if __name__ == '__main__':
    numberkeepinzone = 4
    numberrecoveryzone = 3
    pointlist = [[] for _ in range((numberkeepinzone + numberrecoveryzone))]

    # LEFT DOWN
    pointlist[0] = [40.12915017 - 0.04, -121.4366 - 0.03]
    # LEFT UP
    pointlist[1] = [40.12915017 - 0.04, -120.6866 + 0.03]
    # RIGHT UP
    pointlist[2] = [39.5177 + 0.04, -120.6866 + 0.03]
    # RIHGT DOWN
    pointlist[3] = [39.5177 + 0.04, -121.4366 - 0.03]

    pointlist[4] = [39.9258, -121.2517]
    pointlist[5] = [39.9919, -120.8328]
    pointlist[6] = [39.5794, -121.0448]
    # pointlist[7] = [39.7181, -121.254]
    # pointlist[8] = [39.8012, -121.1]
    pointlist = np.array(pointlist)
    voro = drawvoronoi(pointlist, numberkeepinzone, numberrecoveryzone)
    voro.voronoialgo()

    plt.xlim([pointlist[2][0] - 0.05, pointlist[1][0] + 0.05])
    plt.ylim([pointlist[0][1] - 0.05, pointlist[1][1] + 0.05])
    # plt.xlim([pointlist[2][0], pointlist[1][0]])
    # plt.ylim([pointlist[0][1], pointlist[1][1]])

    plt.title('Voronoi -> Delaunay')
    plt.savefig('voro' + str(numberrecoveryzone) + 'pick3.png')
    plt.show()








    #