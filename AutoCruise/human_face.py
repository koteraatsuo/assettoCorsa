import cv2
import numpy as np


def to_grayscale(path):
    img = cv2.imread(path)
    grayed = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return grayed


# img = cv2.imread(IMAGE_PATH)
# img.shape

def to_matplotlib_format(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def binary_threshold(path):
    img = cv2.imread(path)
    grayed = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    under_thresh = 105
    upper_thresh = 145
    maxValue = 255
    th, drop_back = cv2.threshold(grayed, under_thresh, maxValue, cv2.THRESH_BINARY)
    th, clarify_born = cv2.threshold(grayed, upper_thresh, maxValue, cv2.THRESH_BINARY_INV)
    merged = np.minimum(drop_back, clarify_born)
    return merged

def mask_blue(path):
    img = cv2.imread(path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    blue_min = np.array([100, 170, 200], np.uint8)
    blue_max = np.array([120, 180, 255], np.uint8)

    blue_region = cv2.inRange(hsv, blue_min, blue_max)
    white = np.full(img.shape, 255, dtype=img.dtype)
    background = cv2.bitwise_and(white, white, mask=blue_region)  # detected blue area becomes white

    inv_mask = cv2.bitwise_not(blue_region)  # make mask for not-blue area
    extracted = cv2.bitwise_and(img, img, mask=inv_mask)

    masked = cv2.add(extracted, background)

    return masked

def blur(img):
    filtered = cv2.GaussianBlur(img, (11, 11), 0)
    return filtered

def detect_contour(path, min_size):
    contoured = cv2.imread(path)
    forcrop = cv2.imread(path)

    # make binary image
    birds = binary_threshold_for_birds(path)
    birds = cv2.bitwise_not(birds)

    # detect contour
    im2, contours, hierarchy = cv2.findContours(birds, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    crops = []
    # draw contour
    for c in contours:
        if cv2.contourArea(c) < min_size:
            continue

        # rectangle area
        x, y, w, h = cv2.boundingRect(c)
        x, y, w, h = padding_position(x, y, w, h, 5)

        # crop the image
        cropped = forcrop[y:(y + h), x:(x + w)]
        cropped = resize_image(cropped, (210, 210))
        crops.append(cropped)

        # draw contour
        cv2.drawContours(contoured, c, -1, (0, 0, 255), 3)  # contour
        cv2.rectangle(contoured, (x, y), (x + w, y + h), (0, 255, 0), 3)  #rectangle contour

    return contoured, crops


def padding_position(x, y, w, h, p):
    return x - p, y - p, w + p * 2, h + p * 2


def various_contours(path):
    color = cv2.imread(path)
    grayed = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(grayed, 218, 255, cv2.THRESH_BINARY)
    inv = cv2.bitwise_not(binary)    
    _, contours, _ = cv2.findContours(inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if cv2.contourArea(c) < 90:
            continue

        epsilon = 0.01 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(color, c, -1, (0, 0, 255), 3)
        cv2.drawContours(color, [approx], -1, (0, 255, 0), 3)

    plt.imshow(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))


# load image, change color spaces, and smoothing
img = cv2.imread('lena.jpg')
img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img_HSV = cv2.GaussianBlur(img_HSV, (9, 9), 3)

# detect tulips
img_H, img_S, img_V = cv2.split(img_HSV)
_thre, img_flowers = cv2.threshold(img_H, 140, 255, cv2.THRESH_BINARY)
cv2.imwrite('tulips_mask.jpg', img_flowers)

# find tulips
contours, hierarchy = cv2.findContours(img_flowers, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

for i in range(0, len(contours)):
    if len(contours[i]) > 0:

        # remove small objects
        if cv2.contourArea(contours[i]) < 500:
            continue

        rect = contours[i]
        x, y, w, h = cv2.boundingRect(rect)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), 10)

# save
cv2.imwrite('tulips_boundingbox.jpg', img)