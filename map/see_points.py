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
    pixPoints = []
    for jj in range(pointNum):
        for ii in range(pointNum): 
            h = jj*height/pointNum+height/pointNum/2
            w = ii*width/pointNum+width/pointNum/2
            mappArray[h][w] = 25
    f = open("test.pgm", 'w')
    f.write('P2\n')
    f.write('{} {}\n'.format(width, height))
    f.write('255\n')
    for j in range(height):
        for i in range(width):
            f.write('{}\n'.format(mappArray[j][i]))
    f.close()

search_grid('cropped.pgm', 4)
