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
        i[0]-=xOff
        i[1]-=yOff
        result.append(i)
    return result


x = search_grid('cropped.pgm', 100)
x = pix_to_pos(x, .05, -10.65, -5.45)
print(x)