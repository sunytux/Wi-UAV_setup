FILE(GLOB SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/baseStation"
    "include/*.h"
    "include/*.hpp"
    "src/*.cpp"
)

add_executable(baseStation ${SOURCE_FILES})

target_include_directories(baseStation 
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(baseStation
    djiosdk-core
    djiosdk-wrapper
    djiosdk-platform
)
