project(RenderCore_Whitted)

file(GLOB sources "*.cpp")
file(GLOB headers "*.h")
add_library(${PROJECT_NAME} SHARED ${sources} ${headers})
target_compile_definitions(${PROJECT_NAME} PRIVATE COREDLL_EXPORTS=1)

target_compile_options(${PROJECT_NAME} PRIVATE $<$<BOOL:${MSVC}>:/openmp>)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<BOOL:${MSVC}>>:-fopenmp -Ofast>)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<BOOL:${MSVC}>>:-Wall -Wextra -Wno-unknown-pragmas -Wno-unused-parameter>)

target_link_libraries(${PROJECT_NAME} PRIVATE RenderSystem)
set_target_properties(${PROJECT_NAME} PROPERTIES
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF)
if(BUILD_SHARED_LIBS)
	install(TARGETS ${PROJECT_NAME})
endif()

