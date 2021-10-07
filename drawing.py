
import cv2


####################
#                  #
#   Coordination   #
#                  #
#    +－－－－→ y   #
#    |             #
#    |             #
#    ↓             #
#    x             #
#                  #
####################


def drawSquare(image: list, x: int, y: int, width: int, height: int, color: tuple, isFilled = False) -> list:

    if isFilled:
        thickness = -1
    else:
        thickness = 1

    return cv2.rectangle(image, (y, x), (y+width-1, x+height-1), color, thickness)

