import numpy as np
import vtk


def numpy_type_as_vtk_type(numpy_type):
    vtk_type_by_numpy_type = {
        np.uint8: vtk.VTK_UNSIGNED_CHAR,
        np.uint16: vtk.VTK_UNSIGNED_SHORT,
        np.uint32: vtk.VTK_UNSIGNED_INT,
        np.uint64: vtk.VTK_UNSIGNED_LONG if vtk.VTK_SIZEOF_LONG == 64 else vtk.VTK_UNSIGNED_LONG_LONG,
        np.int8: vtk.VTK_CHAR,
        np.int16: vtk.VTK_SHORT,
        np.int32: vtk.VTK_INT,
        np.int64: vtk.VTK_LONG if vtk.VTK_SIZEOF_LONG == 64 else vtk.VTK_LONG_LONG,
        np.float32: vtk.VTK_FLOAT,
        np.float64: vtk.VTK_DOUBLE
    }
    return vtk_type_by_numpy_type[numpy_type]


def numpy_image_as_vtk_image_data(source_numpy_image):
    """
    :param source_numpy_image: source array with 2-3 dimensions. If used, the third dimension represents the channel count.
    Note: Channels are flipped, i.e. source is assumed to be BGR instead of COLOR (which works if you're using cv2.imread function to read three-channel images)
    Note: Assumes array value at [0,0] represents the upper-left pixel.
    :type source_numpy_image: np.ndarray
    :return: vtk-compatible image, if conversion is successful. Raises exception otherwise
    :rtype vtk.vtkImageData
    """
    from vtk.util import numpy_support

    if len(source_numpy_image.shape) > 2:
        channel_count = source_numpy_image.shape[2]
    else:
        channel_count = 1

    output_vtk_image = vtk.vtkImageData()
    output_vtk_image.SetDimensions(source_numpy_image.shape[1], source_numpy_image.shape[0], channel_count)

    vtk_datatype = numpy_type_as_vtk_type(source_numpy_image.dtype.type)

    source_numpy_image = np.flipud(source_numpy_image)

    # Note: don't flip (take out next two lines) if input is COLOR.
    # Likewise, BGRA->RGBA would require a different reordering here.
    if channel_count > 1:
        source_numpy_image = np.flip(source_numpy_image, 2)

    depth_array = numpy_support.numpy_to_vtk(source_numpy_image.ravel(), deep=True, array_type=vtk_datatype)
    depth_array.SetNumberOfComponents(channel_count)
    output_vtk_image.SetSpacing([1, 1, 1])
    output_vtk_image.SetOrigin([-1, -1, -1])
    output_vtk_image.GetPointData().SetScalars(depth_array)

    output_vtk_image.Modified()
    return output_vtk_image


def update_vtk_image(target_vtk_image, source_numpy_image):
    from vtk.util import numpy_support
    if len(source_numpy_image.shape) > 2:
        channel_count = source_numpy_image.shape[2]
    else:
        channel_count = 1
    vtk_datatype = numpy_type_as_vtk_type(source_numpy_image.dtype.type)
    depth_array = numpy_support.numpy_to_vtk(source_numpy_image.ravel(), deep=True, array_type=vtk_datatype)
    depth_array.SetNumberOfComponents(channel_count)
    target_vtk_image.GetPointData().SetScalars(depth_array)
    target_vtk_image.Modified()


def convert_to_viewable_depth(depth_image, near_clipping_distance=0.2, far_clipping_distance=3.0):
    clipping_range = far_clipping_distance - near_clipping_distance
    raw_numpy_depth_copy = depth_image.copy()
    raw_numpy_depth_copy[depth_image > far_clipping_distance] = far_clipping_distance
    return 255 - ((raw_numpy_depth_copy - near_clipping_distance) * 255 / clipping_range).astype(np.uint8)
