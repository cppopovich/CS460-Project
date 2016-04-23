'''
Find places on the map to search
205 is grey on map, 0 is black, 254 and 255 are the whites
'''
def search_grid(m, pointNum):
    mapp = open(m, 'r')
    mappArray = []
    mapp.next()
    mapp.next()
    width, height = mapp.next().split(' ')
    width = int(width)
    height = int(height)
    max = int(mapp.next())
    for j in range(height):
        mappArray.append([])
        for i in range(width):
            mappArray[j].append(int(mapp.next()))
    if pointNum >= 100:
        pointNum = 10
    else:
        for a in range(9):
            if (a+1)**2 > pointNum:
                pointNum = a

    pixPoints = []
    for jj in range(pointNum):
        for ii in range(pointNum):
            h = jj*height/pointNum
            w = ii*width/pointNum
            if(mappArray[h][w] > 205):
                pixPoints.append([w, h])
    return pixPoints

def pix_to_pos(pixes, res, xOff, yOff):
    result = []
    for i in pixes:
        i[0]*=.05
        i[1]*=.05
        i[0]+=xOff
        i[1]+=yOff
        result.append({'x':i[0], 'y':-i[1]})
    return result

print(pix_to_pos(search_grid('cropped.pgm', 16), .05, -10.65, -5.45))

#print(pix_to_pos([[1879, 2106]], .05, -100.000000, -100.000000))
#print(pix_to_pos([[95, 213]], .05, -10.65, -5.45))