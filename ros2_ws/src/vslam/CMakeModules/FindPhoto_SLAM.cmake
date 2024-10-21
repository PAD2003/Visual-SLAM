# Try to find Photo_SLAM
# Set alternative paths to search for using Photo_SLAM_DIR
# Once done this will define
# You should ensure your Photo_SLAM can run correctly
#
# To help the search Photo_SLAM_ROOT_DIR environment variable as the path to Photo_SLAM root folder
#  e.g. `set( Photo_SLAM_ROOT_DIR=~/Photo_SLAM) `

# TODO: Set path to masked-photo-slam
set(Photo_SLAM_ROOT_DIR "/home/ducpa/Project/slam3d/Photo-SLAM")
set(ORB_SLAM3_ROOT_DIR "${Photo_SLAM_ROOT_DIR}/ORB-SLAM3")

# message(${Photo_SLAM_ROOT_DIR})
# message(${Photo_SLAM_ROOT_DIR}/include)
# message(${Photo_SLAM_ROOT_DIR}/viewer)

# Find Photo_SLAM
find_path(Photo_SLAM_INCLUDE_DIR NAMES gaussian_mapper.h
          PATHS ${Photo_SLAM_ROOT_DIR}/include ${Photo_SLAM_ROOT_DIR}/viewer)

# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES ORB-SLAM3/include/System.h
          PATHS ${Photo_SLAM_ROOT_DIR})

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib)

# Find built-in DBoW2
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR})

find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib)

# Find simple_knn
find_path(simple_knn_INCLUDE_DIR NAMES third_party/simple-knn/simple_knn.h
       PATHS ${Photo_SLAM_ROOT_DIR})

find_library(simple_knn_LIBRARY NAMES simple_knn libsimple_knn
            PATHS ${Photo_SLAM_ROOT_DIR}/lib)

# Find cuda_rasterizer
find_library(cuda_rasterizer_LIBRARY NAMES cuda_rasterizer libcuda_rasterizer
             PATHS ${Photo_SLAM_ROOT_DIR}/lib)

# FInd imgui
find_library(imgui_LIBRARY NAMES imgui libimgui
             PATHS ${Photo_SLAM_ROOT_DIR}/lib)

# Find gaussian_mapper
find_path(gaussian_mapper_INCLUDE_DIR NAMES gaussian_mapper.h
          PATHS ${Photo_SLAM_ROOT_DIR}/include)

find_library(gaussian_mapper_LIBRARY NAMES gaussian_mapper libgaussian_mapper
             PATHS ${Photo_SLAM_ROOT_DIR}/lib)

# Find Photo_SLAM_gaussian_viewer
find_path(gaussian_viewer_INCLUDE_DIR NAMES imgui_viewer.h
          PATHS ${Photo_SLAM_ROOT_DIR}/viewer)

find_library(gaussian_viewer_LIBRARY NAMES gaussian_viewer libPgaussian_viewer
             PATHS ${Photo_SLAM_ROOT_DIR}/lib)


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set Photo_SLAM_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Photo_SLAM  DEFAULT_MSG
                                  Photo_SLAM_INCLUDE_DIR ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY
                                  simple_knn_LIBRARY cuda_rasterizer_LIBRARY
                                  gaussian_mapper_LIBRARY gaussian_mapper_INCLUDE_DIR gaussian_viewer_LIBRARY gaussian_viewer_INCLUDE_DIR)

mark_as_advanced(Photo_SLAM_INCLUDE_DIR)

set(Photo_SLAM_LIBRARIES "${ORB_SLAM3_LIBRARY}" "${DBoW2_LIBRARY}" "${g2o_LIBRARY}" "${simple_knn_LIBRARY}" "${cuda_rasterizer_LIBRARY}" "${imgui_LIBRARY}" "${gaussian_mapper_LIBRARY}" "${gaussian_viewer_LIBRARY}")
set(Photo_SLAM_INCLUDE_DIRS "${Photo_SLAM_INCLUDE_DIR}" "${ORB_SLAM3_INCLUDE_DIR}" "${DBoW2_INCLUDE_DIR}" "${gaussian_mapper_INCLUDE_DIR}" "${gaussian_viewer_INCLUDE_DIR}")
