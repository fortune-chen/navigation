cmake_minimum_required(VERSION 3.0.2)
project(utils)

aux_source_directory(./ SRC)

add_library(${PROJECT_NAME} STATIC ${SRC})

# install(TARGETS
#         ${PROJECT_NAME}
#         ARCHIVE DESTINATION ./lib
#         LIBRARY DESTINATION ./lib
#         RUNTIME DESTINATION ./bin
# )
