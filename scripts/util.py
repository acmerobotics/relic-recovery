import cv2


def smart_hsv_range(src, lower_hsv, upper_hsv):
    if lower_hsv[0] > upper_hsv[0]:
        mask1 = cv2.inRange(src, lower_hsv, (180, upper_hsv[1], upper_hsv[2]))
        mask2 = cv2.inRange(src, (0, lower_hsv[1], lower_hsv[2]), upper_hsv)
        return cv2.bitwise_or(mask1, mask2)
    else:
        return cv2.inRange(src, lower_hsv, upper_hsv)


def resize_min_dim(src, min_dim):
    height, width = src.shape[:2]
    if height < width:
        new_height = min_dim
        new_width = int(width / height * new_height)
    else:
        new_width = min_dim
        new_height = int(height / width * new_width)
    return cv2.resize(src, (new_width, new_height), interpolation=cv2.INTER_CUBIC)