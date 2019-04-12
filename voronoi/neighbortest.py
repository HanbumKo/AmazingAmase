import scipy.spatial
import numpy
import pylab

def find_neighbors(pindex, triang):
    a = triang.vertex_neighbor_vertices[1][
    triang.vertex_neighbor_vertices[0][pindex]:
    triang.vertex_neighbor_vertices[0][pindex + 1]]
    print(a)
    print(triang.vertex_neighbor_vertices[0][pindex])
    print(triang.vertex_neighbor_vertices[0][pindex+1])
    return a

x_list = numpy.random.random(7)
y_list = numpy.random.random(7)

p_list = [[  39.82822367, -121.02163792],
 [  39.5625,     -121.4666    ],
 [  39.6406,     -120.6566    ],
 [  39.5794,     -121.0448    ],
 [  39.5577,     -120.6566    ],
 [  39.5577,     -121.4666    ],
 [  39.5794,     -121.0448    ]]

for i in range(len(p_list)):
    x_list[i] = p_list[i][0]
    y_list[i] = p_list[i][1]

tri = scipy.spatial.Delaunay(numpy.array([[x,y] for x,y in zip(x_list, y_list)]))

for j in range(len(p_list)):
    pylab.subplot(2, len(p_list)-3, j+1)
    pindex = j

    neighbor_indices = find_neighbors(pindex,tri)
    print(neighbor_indices)
    for i in range(len(x_list)):
        pylab.triplot(x_list, y_list, tri.simplices.copy())

    pylab.plot(x_list, y_list, 'b.')
    pylab.plot(x_list[pindex], y_list[pindex], 'dg')
    pylab.plot([x_list[i] for i in neighbor_indices],
               [y_list[i] for i in neighbor_indices], 'ro')

pylab.show()