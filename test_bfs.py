import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw

omap = np.flip(np.transpose(np.loadtxt('newmap20210414011031.txt')), 1)


def bfs(graph, start):
    # maintain a queue of paths
    queue = []
    # visited = set([start])
    visited = []
    # push the first path into the queue
    queue.append(start)  # [[25,25]]
    # queue= collections.deque(start)
    visited.append(start)
    w = []
    l = 0
    while len(queue) > 0:
        # get the first path from the queue

        path = queue.pop(0)

        if (isinstance(path[0], int)):
            p = path
            l = 1
        else:
            p = path[-1]
            l = 0

        # xx.append(path)
        # print(new_path)
        # get the last node from the path
        # node = path[-1]
        # new_path = []
        # path found

        x = p[0]
        y = p[1]

        # enumerate all adjacent nodes, construct a new path and push it into the queue
        # new_path= list()
        # node x+1 y
        # print(x)
        # print(y)
        if x + 1 < len(graph) - 3 and [x + 1, y] not in visited and graph[x + 1, y] != 3:
            if l == 1:
                q = [path, [x + 1, y]]
                queue.append(q)
                if graph[x + 1, y] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])
                    i = i + 1
                new.append([x + 1, y])
                queue.append(new)
                if graph[x + 1, y] == 1:
                    # print("ccc")

                    return new
            # new_path.append([x+1,y])

            visited.append([x + 1, y])
        # node x+1 y+1
        if x + 1 < len(graph) - 3 and y + 1 < len(graph[0]) and [x + 1, y + 1] not in visited and graph[
            x + 1, y + 1] != 3:
            if (l == 1):
                q = []
                q.append(path)
                q.append([x + 1, y + 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x + 1, y + 1] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x + 1, y + 1])
                queue.append(new)
                if graph[x + 1, y + 1] == 1:
                    # print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x + 1, y + 1])
        # node x y+1
        if y + 1 < len(graph[0]) and [x, y + 1] not in visited and graph[x, y + 1] != 3:

            if (l == 1):
                q = []
                q.append(path)
                q.append([x, y + 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x, y + 1] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x, y + 1])
                queue.append(new)
                if graph[x, y + 1] == 1:
                    # print("ccc")

                    return new
            visited.append([x, y + 1])
        # node x-1 y+1
        if x - 1 > -1 and y + 1 < len(graph[0]) and [x - 1, y + 1] not in visited and graph[
            x - 1, y + 1] != 3:
            if (l == 1):
                q = []
                q.append(path)
                q.append([x - 1, y + 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x - 1, y + 1] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y + 1])
                queue.append(new)
                if graph[x - 1, y + 1] == 1:
                    # print("ccc")

                    return new
            visited.append([x - 1, y + 1])

        # node x-1 y
        if x - 1 > -1 and [x - 1, y] not in visited and graph[x - 1, y] != 3 and graph[x - 2, y] != 3:
            if (l == 1):
                q = []
                q.append(path)
                q.append([x - 1, y])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x - 1, y] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y])
                queue.append(new)
                if graph[x - 1, y] == 1:
                    # print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x - 1, y])
        # node x-1 y-1
        if x - 1 > -1 and y - 1 > -1 and [x - 1, y - 1] not in visited and graph[x - 1, y - 1] != 3:
            if (l == 1):
                q = []
                q.append(path)
                q.append([x - 1, y - 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x - 1, y - 1] == 1:
                    print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x - 1, y - 1])
                queue.append(new)
                if graph[x - 1, y - 1] == 1:
                    # print("ccc")

                    return new
            visited.append([x - 1, y - 1])

        # node x y-1
        if y - 1 > -1 and [x, y - 1] not in visited and graph[x, y - 1] != 3:

            if (l == 1):
                q = []
                q.append(path)
                q.append([x, y - 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x, y - 1] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x, y - 1])
                queue.append(new)
                if graph[x, y - 1] == 1:
                    # print("ccc")

                    return new
            # new_path.append([x+1,y])
            visited.append([x, y - 1])
        # node x+1 y-1
        if x + 1 < len(graph) and y - 1 > -1 and [x + 1, y - 1] not in visited and graph[
            x + 1, y - 1] != 3:

            # new_path.append([x+1,y-1])
            if (l == 1):
                q = []
                q.append(path)
                q.append([x + 1, y - 1])  # queue.append( path + [x+1,y])
                queue.append(q)
                if graph[x + 1, y - 1] == 1:
                    # print("ccc")

                    return q
            else:
                i = 0
                new = []
                while (i <= len(path) - 1):
                    new.append(path[i])

                    i = i + 1

                new.append([x + 1, y - 1])
                queue.append(new)
                if graph[x + 1, y - 1] == 1:
                    # print("ccc")

                    return new
            visited.append([x + 1, y - 1])
    # print(len(queue))

    return None

def bfs2(occdata, start):
    queue = []
    visited = []
    queue.append(start)
    visited.append(start)
    max_path = 0
    l = 0
    while len(queue) > 0:
        path = queue.pop(0)

        if isinstance(path[0], int):
            p = path
            l = 1
        else:
            p = path[-1]
            l = 0

        x = p[0]
        y = p[1]

        if x + 1 < len(occdata[0]) and y + 1 < len(occdata) and [x + 1, y + 1] not in visited and occdata[x + 1][
            y + 1] != 3:
            if l == 1:
                q = [path, [x + 1, y + 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x + 1][y + 1] == 1:
                    return q
            else:
                new = []
                for j in path:
                    new.append(j)
                new.append([x + 1, y + 1])
                queue.append(new)
            visited.append([x + 1, y + 1])

        if x + 1 < len(occdata[0]) and [x + 1, y] not in visited and occdata[x + 1][y] != 3:
            if l == 1:
                q = [path, [x + 1, y]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x + 1][y] == 1:
                    return q
            visited.append([x + 1, y])

        if x + 1 < len(occdata[0]) and y - 1 < -1 and [x + 1, y - 1] not in visited and occdata[x + 1][y - 1] != 3:
            if l == 1:
                q = [path, [x + 1, y - 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x + 1][y - 1] == 1:
                    return q
            visited.append([x + 1, y - 1])

        if y - 1 < -1 and [x, y - 1] not in visited and occdata[x][y - 1] != 3:
            if l == 1:
                q = [path, [x, y - 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x][y - 1] == 1:
                    return q
            visited.append([x, y - 1])

        if x - 1 < -1 and y - 1 < -1 and [x - 1, y - 1] not in visited and occdata[x - 1][y - 1] != 3:
            if l == 1:
                q = [path, [x - 1, y - 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x - 1][y - 1] == 1:
                    return q
            visited.append([x - 1, y - 1])

        if x - 1 < -1 and [x - 1, y] not in visited and occdata[x - 1][y] != 3:
            if l == 1:
                q = [path, [x - 1, y]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x - 1][y] == 1:
                    return q
            visited.append([x - 1, y])

        if x - 1 < -1 and y + 1 < len(occdata) and [x - 1, y + 1] not in visited and occdata[x - 1][y + 1] != 3:
            if l == 1:
                q = [path, [x - 1, y + 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x - 1][y + 1] == 1:
                    return q
            visited.append([x - 1, y + 1])

        if y + 1 < len(occdata) and [x, y + 1] not in visited and occdata[x][y + 1] != 3:
            if l == 1:
                q = [path, [x, y + 1]]
                if len(q) > max_path:
                    max_path = len(q)
                    queue.append(q)
                if occdata[x][y + 1] == 1:
                    return q
            visited.append([x, y + 1])

points = bfs(omap, [19, 45])

for i in points:
    omap[i[0]][i[1]] = 0

print(points)

plt.grid()
plt.imshow(omap, origin='lower')
plt.draw_all()
plt.pause(0.00000000001)
