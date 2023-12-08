import math

def cigar_ship(length, diameter, fineness=8):
    vertices = []
    
    current_z = length * 0.5
    for i in range(fineness):
        theta = math.radians(360/fineness * i)
        r = diameter * 0.5
        point_x = r * math.cos(theta)
        point_y = r * math.sin(theta)
        point_z = current_z
        vertices.append([point_x, point_y, point_z])

    current_z = -length * 0.5
    for i in range(fineness):
        theta = math.radians(360/fineness * i)
        r = diameter * 0.5
        point_x = r * math.cos(theta)
        point_y = r * math.sin(theta)
        point_z = current_z
        vertices.append([point_x, point_y, point_z])

    lines = []
    for i in range(1, fineness):
        lines.append([i, i+1])

    lines.append([fineness, 1])

    for i in range(fineness + 1, fineness * 2):
        lines.append([i, i+1])

    lines.append([fineness * 2, fineness + 1])

    for i in range(fineness):
        lines.append([i, i+fineness])

    f = open("cigarmodel.mdl", "w")
    for v in vertices:
        text = "V|" + str(v[0]) + ", " + str(v[1]) + ", " + str(v[2]) + "\n"
        f.write(text)

    for l in lines:
        text = "L|" + str(l[0]) + ", " + str(l[1]) + "\n"
        f.write(text)

    f.close()

cigar_ship(300, 50, 8)
