#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/PI.h>
#include "tutorial_shared_path.h"

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace igl;

    // Read in inputs as double precision floating point meshes
    MatrixXi F;
    MatrixXd V;
    read_triangle_mesh(TUTORIAL_SHARED_PATH "/armadillo.obj",V,F);
        
    // Options
    int frames_per_rotation = 100;
    float look_at_radius = 4.f;

    // Set mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_face_based(true);
    viewer.core().is_animating = true;

    // Set rotation incremental angles (in [rad])
    float theta = 0.f;
    float theta_delta = 2.f*igl::PI / static_cast<float>(frames_per_rotation);

    // Rotate camera around the mesh
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool {
        theta += theta_delta;
        viewer.core().camera_eye = Eigen::Vector3f(
            look_at_radius*sin(theta), 0.f, cos(theta)*look_at_radius);
        return false;
    };

    viewer.launch();
}
