import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.spatial import Voronoi
import math
import time
import copy

def voronoi_finite_polygons_2d(vor, radius=None):
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

def findtouchkeepinzone(src, dst):
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
            standard = (point_list[1][1] - src[1])/(point_list[1][0] - src[0])
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
            standard = (point_list[0][1] - src[1]) / (point_list[0][0] - src[0])
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
            standard = (point_list[2][1] - src[1]) / (point_list[2][0] - src[0])
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
            standard = (point_list[3][1] - src[1]) / (point_list[3][0] - src[0])
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
def findpoint(polygon, vertex_list_element):
    new_points = []
    # 각각 함수를 구하고 keepinzone의 좌표를 넣어서 해당 점을 구한다.
    # 구한 점이 keepinzone의 범위에 있고 idx와 끝점의 중간에 있으면 선택
    '''
    a0 = [40.0928, -121.4366]
    a1 = [40.0928, -120.6866]
    a2 = [39.5177, -120.6866]
    a3 = [39.5177, -121.4366]
    '''
    # sx = 39.5177
    # lx = 40.0928
    # sy = -121.4366
    # ly = -120.6866

    # sx = point_list[2][0]
    # lx = point_list[0][0]
    # sy = point_list[0][1]
    # ly = point_list[2][1]

    print(polygon)
    print(vertex_list_element)
    vertex_list_element.sort()
    print("sorted",vertex_list_element)
    # idx1 = 0
    # target1 = 0
    # idx2 = 0
    # target2 = 0
    # 모든 면은 무조건 2개아니면 0개가 keepinzone과 만난다.
    print("len(vertex_list)",len(vertex_list_element))
    print("polygon",len(polygon))
    print(len(polygon)-len(vertex_list_element))
    # 모이는 점에 들어가 있는 점이 하나일때 => [2]

    if len(vertex_list_element) == 1:
        idx1 = vertex_list_element[0]
        idx2 = vertex_list_element[0]
        target1 = (idx1 - 1) % len(polygon)
        target2 = (idx2 + 1) % len(polygon)
    else:
        # 모이는 점들이 연속적인 점들로 이루어져 있음 => [2,3,4]
        if (vertex_list_element[-1] - vertex_list_element[0]) == (len(vertex_list_element) - 1) :
            idx1 = vertex_list_element[0]
            target1 = (idx1 - 1) % len(polygon)
            idx2 = vertex_list_element[-1]
            target2 = (idx2 + 1) % len(polygon)
        # 모이는 점들이 불연속 적인 점들로 이루어져 있음 => [1,2,4,0]
        else :
            for i in range(len(vertex_list_element)-1):
                if vertex_list_element[i] + 1 != vertex_list_element[i+1]:
                    idx1 = vertex_list_element[i]
                    target1 = idx1 + 1
                    idx2 = vertex_list_element[i+1]
                    target2 = idx2 - 1



    print("idx target : ",idx1, target1, idx2, target2)

    # 각각 idx에 대해서 target point를 이용해서 keepinzone에 닿는 좌표를 찾기
    p1 = findtouchkeepinzone(polygon[idx1], polygon[target1])
    p2 = findtouchkeepinzone(polygon[idx2], polygon[target2])
    '''
    VISUALIZATION
    '''
    # plt.plot(p1[0], p1[1], 'o')
    # plt.plot(p2[0], p2[1], 'o')w2
    # plt.text(p1[0], p1[1], "p1", fontsize=8)
    # plt.text(p2[0] + 0.008, p2[1], "p2", fontsize=8)
    print("p1",p1)
    print("p2",p2)
    polygon_area = []
    for i in range(len(vertex_list_element)):
        polygon_area.append(polygon[vertex_list_element][i].tolist())
        vor_list_for_loiter[i] = copy.deepcopy(polygon_area)
        print("self.vor_list_for_loiter[i]\n", vor_list_for_loiter[i])
    polygon_area.append(p1)
    polygon_area.append(p2)
    print("polygon_area\n",polygon_area)
    return polygon_area

def caldistancebetweentwopoint(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    d = math.sqrt((dx * dx) + (dy * dy))
    return d

def caldistance(points):
    distance = [0,0,0]
    for i in range(len(points)):
        dx = points[(i + 1) % 3][0] - points[i % 3][0]
        dy = points[(i + 1) % 3][1] - points[i % 3][1]

        d = math.sqrt((dx * dx) + (dy * dy))
        distance[i] = d
    return distance

def areasize(points):
    d = caldistance(points)

    s = (d[0] + d[1] + d[2]) / 2
    r1 = (s * (s - d[0]) * (s - d[1]) * (s - d[2])) ** 0.5

    return r1


'''
점 세개를 받아서 가장 작은 각도와 그 다음 작은 각도를 확인하고 로이터 방법 저장
가장 작은 각도와 그 다음 작은 각도를 가지고 있는 인덱스를 저장
저장한 두 값 리턴
'''
def returnangle(points):
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

def findthreetriangle(findpointlist):
    # 꼭지점을 하나만 가지고 있는 경우
    findpointlist = findpointlist.tolist()
    threetriangle = [[],[],[]]
    points = [0, 0, 0]
    if len(findpointlist) == 5:
        print("꼭지점 4개")
        t1 = [findpointlist[0], findpointlist[1], findpointlist[4]]
        t2 = [findpointlist[0], findpointlist[2], findpointlist[4]]
        if areasize(t1) > areasize(t2):
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



'''
point_list[4] = [39.9669, -121.2713]
point_list[5] = [39.872, -120.747]
point_list[6] = [39.6712, -120.7513]
#point_list[7] = [39.7181, -121.254]
#point_list[7] = [39.8012, -121.1]
'''
if __name__ == '__main__':
    number_keepinzone = 4
    number_recoveryzone = 3
    point_list = [[] for _ in range((number_keepinzone + number_recoveryzone))]

    # Scenario = "A"
    # [[40.12915017 - 121.38244271]
    #  [40.12915017 - 121.38244271]
    #  [39.48541255 - 120.73870509]
    #  [39.48541255 - 120.73870509]
    #  [39.96835881 - 121.27003869]
    #  [39.67217196 - 120.75002526]
    #  [39.87130371 - 120.74903138]]
    # scenario = "B"
    # [[40.08915017 - 121.43244271]
    #  [40.08915017 - 120.78870509]
    #  [39.52541255 - 120.78870509]
    #  [39.52541255 - 121.43244271]
    #  [39.81055416 - 121.31201043]
    #  [39.73152219 - 120.8901842]
    #  [39.83938013 - 121.00869938]]




    '''
    practice1
    Lat: 39.9162 Lon: -121.2599 Alt: 1231 m
    Lat: 39.9912 Lon: -120.8633 Alt: 1566 m
    Lat: 39.5939 Lon: -121.0185 Alt: 1329 m
    '''
    '''
    practice2
    Lat: 39.9956 Lon: -121.303 Alt: 1537 m
    Lat: 39.882 Lon: -120.8332 Alt: 1346 m
    Lat: 39.6458 Lon: -121.1823 Alt: 1094 m
    '''
    '''
    practice3
    Lat: 39.9956 Lon: -121.303 Alt: 1537 m
    Lat: 39.882 Lon: -120.8332 Alt: 1346 m
    Lat: 39.6458 Lon: -121.1823 Alt: 1094 m
    '''
    '''
    Lat: 39.8599 Lon: -121.2024 Alt: 1627 m
    Lat: 39.7429 Lon: -120.7441 Alt: 2211 m
    Lat: 39.5796 Lon: -121.2958 Alt: 744 m
    '''
    scenario = "practice3c"
    # LEFT DOWN



    point_list[0] = [40.12915017 - 0.04, -121.4366 - 0.03]
    # LEFT UP
    point_list[1] = [40.12915017 - 0.04, -120.6866 + 0.03]
    # RIGHT UP
    point_list[2] = [39.5177 + 0.04, -120.6866 + 0.03]
    # RIHGT DOWN
    point_list[3] = [39.5177 + 0.04, -121.4366 - 0.03]
    #
    '''
    Lat: 39.9581 Lon: -121.3619 Alt: 1603 m
    Lat: 39.8974 Lon: -120.803 Alt: 1339 m
    Lat: 39.6425 Lon: -120.974 Alt: 1503 m
    '''
    '''
    2C
    Lat: 39.7308 Lon: -121.3346 Alt: 787 m
    Lat: 39.8411 Lon: -121.0084 Alt: 1561 m
    Lat: 39.6734 Lon: -120.8159 Alt: 1829 m
    '''
    '''
    3A
    Lat: 40.0674 Lon: -121.3963 Alt: 1888 m
    Lat: 39.691 Lon: -121.0903 Alt: 1558 m
    Lat: 39.5707 Lon: -120.8562 Alt: 1334 m
    '''
    '''
    3B
    Lat: 39.8599 Lon: -121.1952 Alt: 1712 m
    Lat: 39.7429 Lon: -120.7441 Alt: 2211 m
    Lat: 39.5807 Lon: -121.2958 Alt: 742 m
    '''
    '''
    Lat: 39.8599 Lon: -121.2024 Alt: 1627 m
    Lat: 39.7429 Lon: -120.7441 Alt: 2211 m
    Lat: 39.5796 Lon: -121.2958 Alt: 744 m
    '''

    '''
    Lat: 40.0674 Lon: -121.1162 Alt: 1898 m
    Lat: 39.8875 Lon: -120.747 Alt: 1431 m
    Lat: 39.8124 Lon: -121.1047 Alt: 1716 m
    '''
    '''
    R1A
    Lat: 39.9258 Lon: -121.2517 Alt: 1506 m
    Lat: 39.9919 Lon: -120.8328 Alt: 1758 m
    Lat: 39.5794 Lon: -121.0448 Alt: 1213 m
    '''
    point_list[4] = [39.9258, -121.2517]
    point_list[5] = [39.9919, -120.8328]
    point_list[6] = [39.5794, -121.0448]
    #point_list[7] = [39.7181, -121.254]
    # point_list[8] = [39.8012, -121.1]
    # point_list[9] = [39.6964, -120.727]
    # point_list[10] = [39.6133, -121.077]
    # point_list[11] = [39.7381, -121.354]

    # # LEFT DOWN
    # point_list[0] = [-121.4366, 40.0928]
    # # LEFT UP
    # point_list[1] = [-120.6866, 40.0928]
    # # RIGHT UP
    # point_list[2] = [-120.6866, 39.5177]
    # # RIHGT DOWN
    # point_list[3] = [-121.4366, 39.5177]
    #
    # point_list[4] = [-121.2713, 39.9669]
    # point_list[5] = [-120.747, 39.872]
    # point_list[6] = [-120.7513, 39.6712]
    # point_list[7] = [39.7181, -121.254]
    # point_list[8] = [39.8012, -121.1]
    # point_list[9] = [39.6964, -120.727]
    # point_list[10] = [39.6133, -121.077]
    # point_list[11] = [39.7381, -121.354]

    point_list = np.array(point_list)

    print(point_list)
    # compute Voronoi tesselation
    vor = Voronoi(point_list[4:])

    # plot
    ver = vor.vertices.tolist()
    ver = np.array(ver)
    regions, vertices = voronoi_finite_polygons_2d(vor)
    vor_list_for_loiter = [0 for _ in range(len(ver))]
    print("vor_list_for_loiter\n",vor_list_for_loiter)
    # colorize
    idx = []
    t = 0
    vertex_list = [[] for _ in range(len(point_list[4:]))]
    findpointlist = [0 for _ in range(len(point_list[4:]))]


    sx = point_list[2][0]
    lx = point_list[0][0]
    sy = point_list[0][1]
    ly = point_list[2][1]
    ver_list = []
    '''
    Voronoi 그리기 & keepinzone과 만나는 지점 찾기
    '''
    for region in regions:
        print("-----------",t,"-----------")
        polygon = vertices[region]
        plt.fill(*zip(*polygon), alpha=0.4)
        for i in range(len(ver)):
            '''
            만나는 점이 keepinzone안에 있는지 확인
            만나는 점에서 keepinzone 밖에서 생길 수 있
            '''
            if ver[i].round(5)[0] > sx and ver[i].round(5)[0] < lx:
                if ver[i].round(5)[1] > sy and ver[i].round(5)[1] < ly:
                    idx = np.where(ver[i].round(5) == polygon.round(5))[0]
                    if len(idx) > 0:
                        vertex_list[t].append(idx[0])
                        print("ver\n",[ver[i]])
                        if not ver[i].tolist() in ver_list:
                            ver_list.append(ver[i].tolist())
        # 현재 polygon의 좌표들과 keepinzone안에 있는 좌표의 인덱스를 보냄
        findpointlist[t] = np.array(findpoint(polygon, vertex_list[t]))

        t += 1
        print("self.__ver_list\n",ver_list)

        findpointlist = np.array(findpointlist)

    '''
    보로노이 알고리즘을 통해 나온 영역을 좌표로 선택하기 위함
    keepinzone의 모서리 부분을 findpointlist에 삽입
    '''
    dis = [[0 for _ in range(number_recoveryzone)] for _ in range(number_keepinzone)]
    for i in range(number_keepinzone):
        for j in range(number_recoveryzone):
            dis[i][j] = caldistancebetweentwopoint(point_list[i], point_list[number_keepinzone + j])

    for i in range(number_keepinzone):
        min_d = dis[i].index(min(dis[i]))
        findpointlist[min_d] = np.append(findpointlist[min_d],[point_list[i].tolist()], axis=0)

    '''
    voronoi로 나눈 영역들에 recovery zone 위치 삽입
    '''
    for i in range(number_recoveryzone):
        findpointlist[i] = np.append(findpointlist[i],[point_list[i + number_keepinzone]],axis=0)
        plt.text(point_list[i + number_keepinzone][1], point_list[i + number_keepinzone][0], "{}".format(i), fontsize=8)
        print("findpointlis\n",findpointlist[i])


    print("findpointlist",len(findpointlist))
    for j in range(len(findpointlist)):
        print(findpointlist[j])
        t, points= findthreetriangle(findpointlist[j])
        print("findthreetriangle!!!!\n",t)
        #findthreetriangle(findpointlist[0])
        #for i in range(len(t)):
        for i in range(len(t)):
            print("t[i]")
            print("points\n",points)
            t[i] = np.array(t[i])
            print(t[i])
            t[i] = np.append(t[i], [t[i][0]], axis=0)
            print(t[i])
            #for i in range(len(t[0])):
            plt.plot(t[i][:,1],t[i][:,0], 'b')
            print("points\n",points)
            plt.plot(points[i][1], points[i][0], 'ko')

    # t = findthreetriangle(findpointlist[0])
    # print("findthreetriangle!!!!\n", t)
    # # findthreetriangle(findpointlist[0])
    # # for i in range(len(t)):
    # print("t[0]")
    # for i in range(len(t)):
    #     t[i] = np.array(t[i])
    #     print(t[i])
    #     t[i] = np.append(t[i], [t[i][0]], axis=0)
    #     print(t[i])
    #     # for i in range(len(t[0])):
    #     plt.plot(t[i][:, 1], t[i][:, 0], 'b')

    '''
    보로노이 알고리즘을 통해 나온 영역을 들로네 알고리즘으로 나눔
    '''
    trilist = [0 for _ in range(len(findpointlist))]
    for i in range(len(findpointlist)):
        trilist[i] = Delaunay(findpointlist[i])


    '''
    보로노이 알고리즘을 통해 나온 영역을 들로네 알고리즘으로 나눈 삼각형들의 중점을 표시
    '''
    tripoints = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(findpointlist))]
    loiterway = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(findpointlist))]
    a_degree = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(findpointlist))]
    coord = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(findpointlist))]
    arealist = [[0 for _ in range(len(trilist[i].simplices))] for i in range(len(findpointlist))]
    decide = 0
    angle = 0
    t = 0
    for v in range(len(findpointlist)):
        for i in range(len(findpointlist[v][trilist[v].simplices])):
            temp = []
            x = 0
            y = 0
            for j in range(3):
                temp.append(findpointlist[v][trilist[v].simplices][i][j])
                x += findpointlist[v][trilist[v].simplices][i][j][0]
                y += findpointlist[v][trilist[v].simplices][i][j][1]
            x /= 3
            y /= 3


            # 삼각형의 중점 : x, y
            tripoints[v][i] = findpointlist[v][trilist[v].simplices][i]
            #pointss[i] = np.append(findpointlist[v][trilist[v].simplices][i], [[x, y]], axis=0)

            # trilists[v][i] = Delaunay(pointss[i])

            # 로이터 방식과 기울기
            decide, angle = returnangle(tripoints[v][i])
            if angle != None and abs(angle) == 90:
                angle = 0
            if angle != None and abs(angle) == 180:
                angle = 90
            if angle !=None:
                angle = round(angle,3)
            #angle = round(angle,5)
            area = round(areasize(temp),4)
            loiterway[v][i] = decide
            a_degree[v][i] = angle
            coord[v][i] = [x, y]
            arealist[v][i] = area
            '''
            Visualization 
            '''
            plt.plot(y, x, 'v')
            plt.text(y, x, 'zonenubmer {}\ntrinumber {}\nangle {}\narea {}'.format(v, i, angle, area), fontsize=6)
        t += 0.005

    print("\nloiterway\n",np.array(loiterway))
    print("\na_degree\n",np.array(a_degree))
    print("\ncoord\n",np.array(coord))
    print("\narealist\n",np.array(arealist))


    for i in range(len(findpointlist)):
        plt.triplot(findpointlist[i][:, 1], findpointlist[i][:, 0], trilist[i].simplices.copy())
        pass

    '''
    VISUALIZATION
    '''
    for i in range(len(findpointlist)):
        plt.plot(findpointlist[i][:,1], findpointlist[i][:,0], 'o')
        for j in range(len(findpointlist[i])):
            plt.text(findpointlist[i][j][1], findpointlist[i][j][0]-0.01*i, "{},{}".format(i,j),fontsize = 8)

    # # LEFT DOWN
    # point_list[0] = [40.12915017, -121.4366]
    # # LEFT UP
    # point_list[1] = [40.12915017, -120.6866]
    # # RIGHT UP
    # point_list[2] = [39.5177, -120.6866]
    # # RIHGT DOWN
    # point_list[3] = [39.5177, -121.4366]

    plt.xlim([point_list[0][1] - 0.05, point_list[1][1] + 0.05])
    plt.ylim([point_list[2][0] - 0.05, point_list[0][0] + 0.05])
    # plt.xlim([point_list[2][0], point_list[1][0]])
    # plt.ylim([point_list[0][1], point_list[1][1]])
    print(point_list[1][1], point_list[2][1], point_list[1][0], point_list[0][0])
    plt.title('Voronoi -> Delaunay')
    plt.savefig('voro'+str(number_recoveryzone)+'reverse_'+scenario+'.png')
    plt.show()