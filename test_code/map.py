obstacles = [
    [(0.0,0.0),  (0.0,3.048), (4.6419,3.048), (4.6419, 0.0)],
    [(0.0,1.46),(0.0,1.67),(1.47,1.67),(1.47,1.06),(1.26,1.06),(1.26,1.47)],
    [(3.73,2.39),(4.06,1.86),(4.13,1.79),(3.72,1.34),(3.03,1.94),(3.09,2.03)],
    [(0.295,0.0),(0.295,0.11),(1.52,0.11),(1.52,0.0)],
    [(2.06,0.0),(3.16,0.0),(2.51,0.55)],
    [(2.06,0.0),(3.16,0.0),(2.51,0.55)],
    [(0.0,2.97),(0.62,2.44),(0.69,2.52),(0.1,3.05)],
    [(2.0,3.05),(1.48,2.33),(1.39,2.37),(1.90,3.05)],
    [(2.24,1.83),(2.34,1.83),(2.34,3.05),(2.24,3.05)]
]

segments = []
for o in obstacles:
    px,py = None,None
    first = True
    for (x,y) in o:
        if not first:
            segments.append(((px,py),(x,y)))
        else:
            first_point = x,y
        px,py = x,y
        first = False
    segments.append(((px,py),first_point))
print segments

print "void build_map() {"
for ((x1,y1),(x2,y2)) in segments:
    print "    field.push_back({{%f,%f},{%f,%f}});" % (x1,y1,x2,y2)
    print "    cout << \"Segment blue %f %f %f %f\" << endl;" % (x1,y1,x2,y2)
print "}"
