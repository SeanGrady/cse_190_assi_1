import numpy
from PIL import Image

def convert_map_png_to_3d_array(path_to_img):
    im = Image.open(path_to_img)
    return numpy.asarray(im)

def convert_map_png_to_2d_array(path_to_img):
    world_map_3d_array = convert_map_png_to_3d_array(path_to_img)
    temp_list = []
    for row in range(world_map_3d_array.shape[0]):
        for col in range(world_map_3d_array.shape[0]):
            if world_map_3d_array[row][col][0] > 0:
                temp_list.append(255)
            else:
                temp_list.append(0)
    return numpy.asarray(temp_list).reshape((150, 150))

def draw_path_on_world_map_save(path_to_img, final_path):
    world_map_3d_array = convert_map_png_to_3d_array(path_to_img)
    final_world_map_with_path = numpy.array(world_map_3d_array, copy=True)
    for path_item in final_path:
        row, col = path_item
        final_world_map_with_path[row][col][0] = 255
        final_world_map_with_path[row][col][1] = 0
        final_world_map_with_path[row][col][2] = 0
        final_world_map_with_path[row][col][3] = 255
    new_file_name = path_to_img.split(".")
    new_file_name = new_file_name[0] + "_traversed_a_star." + new_file_name[1]
    Image.fromarray(final_world_map_with_path).save(new_file_name)

def clean_image_and_save(path_to_img):
    im = Image.open(path_to_img)
    img_array = numpy.asarray(im)
    clean_image = numpy.array(img_array, copy=True)
    for row in range(clean_image.shape[0]):
        for col in range(clean_image.shape[0]):
            if clean_image[row][col][0] > 0:
                clean_image[row][col][0] = 255
                clean_image[row][col][1] = 255
                clean_image[row][col][2] = 255
            else:
                clean_image[row][col][0] = 0
                clean_image[row][col][1] = 0
                clean_image[row][col][2] = 0
            clean_image[row][col][3] = 255
    new_file_name = path_to_img.split(".")
    new_file_name = new_file_name[0] + "_clean." + new_file_name[1]
    Image.fromarray(clean_image).save(new_file_name)