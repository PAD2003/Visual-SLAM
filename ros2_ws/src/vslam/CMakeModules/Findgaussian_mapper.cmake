# Try to find gaussian_mapper
# Set alternative paths to search for using gaussian_mapper_DIR
# Once done this will define
# You should ensure your gaussian_mapper can run correctly
#
# To help the search gaussian_mapper_ROOT_DIR environment variable as the path to Photo_SLAM root folder
#  e.g. `set( gaussian_mapper_ROOT_DIR=~/Photo_SLAM) `

# TODO: Set path to masked-photo-slam
set(gaussian_mapper_ROOT_DIR "/home/ducpa/Project/slam3d/Photo-SLAM")

# Find gaussian_mapper
find_path(gaussian_mapper_INCLUDE_DIR NAMES gaussian_mapper.h
          PATHS ${gaussian_mapper_ROOT_DIR}/include)

find_library(gaussian_mapper_LIBRARY NAMES gaussian_mapper libgaussian_mapper
             PATHS ${gaussian_mapper_ROOT_DIR}/lib)


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set gaussian_mapper_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(gaussian_mapper  DEFAULT_MSG
                                  gaussian_mapper_LIBRARY gaussian_mapper_INCLUDE_DIR)

mark_as_advanced(gaussian_mapper_INCLUDE_DIR gaussian_mapper_LIBRARY )

set(gaussian_mapper_LIBRARIES ${gaussian_mapper_LIBRARY} )
set(gaussian_mapper_INCLUDE_DIRS ${gaussian_mapper_INCLUDE_DIR})
