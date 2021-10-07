import cv2
import numpy as np
import glob


def getConstTypesStr() -> list:

    return ['color', 'depth', 'yolov4']


def getConstFps() -> float:

    return 0.5


for typeStr in getConstTypesStr():
    imagesPath = []
    images = []
    size = (0, 0)

    for imagePath in glob.glob('output/' + typeStr + '_*.jpg'):
        imagesPath.append(imagePath)

    imagesPath.sort()

    for imagePath in imagesPath:
        print('Read "{}"'.format(imagePath))

        image = cv2.imread(imagePath)
        height, width, layers = image.shape
        size = (width, height)
        images.append(image)

    if len(images) > 0:
        video = cv2.VideoWriter('output/' + typeStr + '.avi', cv2.VideoWriter_fourcc(*'DIVX'), getConstFps(), size)
        
        for image in images:
            video.write(image)

        video.release()

        print('\n{} video output successfully\n'.format(typeStr))

    else:
        print('\nNo {} image found.\n'.format(typeStr))

