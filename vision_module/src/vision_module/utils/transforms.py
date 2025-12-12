def resize_image(image, target_size):
    return image.resize(target_size)

def normalize_image(image):
    return image / 255.0

def transform_image(image, target_size):
    image = resize_image(image, target_size)
    image = normalize_image(image)
    return image