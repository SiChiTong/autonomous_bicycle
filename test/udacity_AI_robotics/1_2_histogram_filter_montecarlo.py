#!/usr/bin/env python
# Histogram filter - montecarlo localization (2D)

def sense(p, Z, sensor_right, colors):
    q=[]
    for row in range(len(p)):
        q_row=[]
        for col in range(len(p[row])):
            hit = (Z == colors[row][col])
            q_row.append(p[row][col]*(hit * sensor_right + (1-hit) * (1-sensor_right)))
        q.append(q_row)

    #Normalize
    s = 0.0
    for row in range(len(p)):
        s = s + sum(q[row])

    for row in range(len(p)):
        for col in range(len(p[row])):
            q[row][col] = q[row][col] / s

    return q

def move(p, U, p_move):
    uy = U[0]
    ux = U[1]

    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]

    for row in range(len(p)):
        for col in range(len(p[row])):
            aux[row][col] = (p_move * p[(row - uy) % len(p)][(col - ux) % len(p[row])]) + (1-p_move)*p[row][col]

    return aux

def localize(colors,measurements,motions,sensor_right,p_move):
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

    for i in range(len(measurements)):
        p = sense(p, measurements[i], sensor_right, colors)
        p = move(p, motions[i], p_move)

    return p

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'

colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]

measurements = ['G','G','G','G','G']

motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

p = localize(colors,measurements,motions,sensor_right = 0.7, p_move = 0.8)

show(p) # displays your answer