#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/png/writePNG.h>
#include "tutorial_shared_path.h"

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace igl;

    // Read in inputs as double precision floating point meshes
    MatrixXi F;
    MatrixXd V;
    read_triangle_mesh(TUTORIAL_SHARED_PATH "/armadillo.obj",V,F);

    // Set mesh and camera
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    viewer.core().camera_eye = Eigen::Vector3f(0.f, 0.f, 3.f);

    // Launch viewer and draw
    viewer.launch_init();
    viewer.draw();

    // Allocate temporary buffers
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R(1280,800);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G(1280,800);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B(1280,800);
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> A(1280,800);

    // Draw the scene in the buffers
    viewer.core().draw_buffer(viewer.data(),false,R,G,B,A);

    // Save it to a PNG
    igl::png::writePNG(R,G,B,A,"out.png");
    
    viewer.launch_shut();
}
