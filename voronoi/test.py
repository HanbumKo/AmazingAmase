import sys
import numpy as np

coordlist = [[0,7,2,4,10],[1,0,9,3,7],[5,5,0,4,2],[7,1,8,0,9],[3,7,2,5,0]]
startidx = 0
greedyidx = [0 for _ in range(len(coordlist))]

for i in range(len(coordlist)):
    for j in range(len(coordlist[i])):
        if coordlist[i][j] ==0:
            coordlist[i][j] = sys.maxsize

check = [False for _ in range(len(coordlist))]
check[startidx] = True
for i in range(len(coordlist)):
    greedyidx[i] = startidx
    temp = np.array(coordlist[startidx])
    for j in range(len(temp)):
        startidx = np.argmin(temp)
        if not check[startidx]:
            check[startidx] = True
            break
        else:
            temp[startidx] = sys.maxsize

print(greedyidx)