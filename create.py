str = ""
for i in range(3, 4):
    for j in range(0, 4):
        str += "graph[x + %d, y + %d] and " % (i, j)
for i in range(3, 4):
    for j in range(0, 4):
        str += "graph[x + %d, y - %d] and " % (i, j)
for i in range(3, 4):
    for j in range(0, 4):
        str += "graph[x - %d, y + %d] and " % (i, j)
for i in range(3, 4):
    for j in range(0, 4):
        str += "graph[x - %d, y - %d] and " % (i, j)
for i in range(0, 4):
    for j in range(3, 4):
        str += "graph[x + %d, y + %d] and " % (i, j)
for i in range(0, 4):
    for j in range(3, 4):
        str += "graph[x + %d, y - %d] and " % (i, j)
for i in range(0, 4):
    for j in range(3, 4):
        str += "graph[x - %d, y - %d] and " % (i, j)
for i in range(0, 4):
    for j in range(3, 4):
        str += "graph[x + %d, y - %d] and " % (i, j)
print(str)
