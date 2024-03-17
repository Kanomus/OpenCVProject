import numpy


verticalDifferentialKernel=numpy.array([[ 1, 2, 1],
                                        [ 0, 0, 0],
                                        [-1,-2,-1]])

horizontalDifferentialKernel=numpy.transpose(verticalDifferentialKernel)

def detectedges(img,padding=0,stride=1,threshold=10):
    kernx=verticalDifferentialKernel.shape[0]
    kerny=verticalDifferentialKernel.shape[0]
    imgx=img.shape[0]
    imgy=img.shape[1]
    outx=int((imgx+(2*padding)-kernx)/stride)+1
    outy=int((imgy+(2*padding)-kerny)/stride)+1

    hout=numpy.zeros((outx,outy))
    vout=numpy.zeros((outx,outy))

    if padding!=0:
        paddedimg=numpy.zeros((imgx+(2*padding),imgy+(2*padding)))
        paddedimg[int(padding):int(-1*padding),int(padding):int(-1*padding)] = img
    else:
        paddedimg=img

    for y in range(img.shape[1]):
        if y>img.shape[1]-kerny:
            break
        if y%stride==0:
            for x in range(img.shape[0]):
                if x>img.shape[0]-kernx:
                    break
                try:
                    if x%stride==0:
                        outarray=numpy.multiply(horizontalDifferentialKernel,paddedimg[x:x+kernx,y:y+kerny])
                        if(numpy.sum(outarray)>threshold):
                            hout[x][y]=numpy.sum(outarray)
                        outarray=numpy.multiply(verticalDifferentialKernel,paddedimg[x:x+kernx,y:y+kerny])
                        if(numpy.sum(outarray)>threshold):
                            vout[x][y]=numpy.sum(outarray)
                except:
                    break

    '''for y in range(0, img.shape[1] - kerny + 1, stride):  # Updated loop bounds
        for x in range(0, img.shape[0] - kernx + 1, stride):  # Updated loop bounds
            try:
                outarray = numpy.multiply(horizontalDifferentialKernel, paddedimg[x:x+kernx, y:y+kerny])
                hout[int(x/stride), int(y/stride)] = numpy.sum(outarray)
                outarray = numpy.multiply(verticalDifferentialKernel, paddedimg[x:x+kernx, y:y+kerny])
                vout[int(x/stride), int(y/stride)] = numpy.sum(outarray)
            except:
                break
'''
    out=numpy.add(hout,vout)

    return out