get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

SET(CONGLI_FILES
    render_and_save
    rotate_camera
    visualize_sdf
)

foreach(app ${CONGLI_FILES})
    add_executable(${app} ${app}.cpp)
    target_link_libraries(${app} igl::core igl::opengl igl::opengl_glfw igl::png tutorials)
endforeach(app)
