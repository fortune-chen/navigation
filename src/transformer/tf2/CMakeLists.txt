cmake_minimum_required(VERSION 3.10)
project(tf2)

add_compile_options(-fPIC)

#CPP Libraries
add_library(${PROJECT_NAME} STATIC
        src/cache.cpp
        src/buffer_core.cpp
        src/static_cache.cpp
)
