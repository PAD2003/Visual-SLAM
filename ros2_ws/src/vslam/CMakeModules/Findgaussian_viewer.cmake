# Try to find Photo_SLAM_gaussian_viewer
# Set alternative paths to search for using Photo_SLAM_gaussian_viewer_DIR
# Once done this will define
# You should ensure your Photo_SLAM_gaussian_viewer can run correctly
#
# To help the search gaussian_viewer_ROOT_DIR environment variable as the path to Photo_SLAM root folder
#  e.g. `set( gaussian_viewer_ROOT_DIR=~/Photo_SLAM) `

# TODO: Set path to masked-photo-slam
set(gaussian_viewer_ROOT_DIR "/home/ducpa/Project/slam3d/Photo-SLAM")

# message(${gaussian_viewer_ROOT_DIR})
# message(${gaussian_viewer_ROOT_DIR}/viewer)

# Find Photo_SLAM_gaussian_viewer
find_path(gaussian_viewer_INCLUDE_DIR NAMES imgui_viewer.h
          PATHS ${gaussian_viewer_ROOT_DIR}/viewer)

find_library(gaussian_viewer_LIBRARY NAMES gaussian_viewer libPgaussian_viewer
             PATHS ${gaussian_viewer_ROOT_DIR}/lib)


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set gaussian_viewer_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(gaussian_viewer  DEFAULT_MSG
                                  gaussian_viewer_LIBRARY gaussian_viewer_INCLUDE_DIR)

mark_as_advanced(gaussian_viewer_INCLUDE_DIR gaussian_viewer_LIBRARY )

set(gaussian_viewer_LIBRARIES ${gaussian_viewer_LIBRARY} )
set(gaussian_viewer_INCLUDE_DIRS ${gaussian_viewer_INCLUDE_DIR})
