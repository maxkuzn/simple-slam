get_filename_component(LIB_INCLUDE_PATH ".." ABSOLUTE)
get_filename_component(LIB_PATH "." ABSOLUTE)

file(GLOB_RECURSE LIB_CXX_SOURCES ${LIB_PATH}/*.cpp)
file(GLOB_RECURSE LIB_HEADERS ${LIB_PATH}/*.hpp)

add_library(slam STATIC ${LIB_CXX_SOURCES} ${LIB_HEADERS})
target_include_directories(slam PUBLIC ${LIB_INCLUDE_PATH})
# target_link_libraries(toucan PUBLIC pthread)
# target_link_libraries(toucan PUBLIC twist)
target_link_libraries(slam ${THIRD_PARTY_LIBS})

