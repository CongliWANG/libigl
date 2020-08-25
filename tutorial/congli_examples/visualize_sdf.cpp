#include <igl/copyleft/marching_cubes.h>
#include <igl/opengl/glfw/Viewer.h>
#include "tutorial_shared_path.h"

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace igl;
    
    int min_argc = 2;
    std::string sdf_string;
    if (argc < min_argc) {
        std::cout << "Error: No enough inputs." << std::endl;
        std::cout << "Usage: visualize_sdf <sdf_file.txt>" << std::endl;
        return -1;
    } else {
        sdf_string = argv[1];
    }
    
    // Read SDF from file
    std::ifstream file;
    int N;
    int Ns[3];
    double full_length[3], half_length[3];
    std::vector<double> raw_data;
    if (!std::ifstream(sdf_string)) { // Check if file exists
        std::cout << "File '" << sdf_string << "' does not exist." << std::endl;
        return -1;
    } else {
        std::ifstream file;
        file.open(sdf_string.c_str(), std::ifstream::in);
        
		// dimension
		file >> Ns[0];
		file >> Ns[1];
		file >> Ns[2];
        // voxel size
        double voxel_size[3];
		file >> voxel_size[0];
		file >> voxel_size[1];
		file >> voxel_size[2];

        // Parsed parameters
        for (int i = 0; i < 3; ++i) {
            full_length[i] = Ns[i] * voxel_size[i];
            half_length[i] = full_length[i]/2.;
        }

        N = Ns[0]*Ns[1]*Ns[2];
        raw_data.resize(N);
        for (int i = 0; i < N; ++i) file >> raw_data[i];
        file.close();
        std::cout << "File '" << sdf_string << "' has been loaded." << std::endl;
    }
    
    // Initialize S and B from file data
    VectorXd S = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(raw_data.data(), raw_data.size());
    VectorXd B = S;
    std::for_each(B.data(),B.data()+B.size(),[](double& b){b=(b>0?1:(b<0?-1:0));});
    
    // Create grid
    std::cout << "Creating grid..." << std::endl;
    MatrixXd GV(N, 3);
    for (int zi = 0; zi < Ns[2]; ++zi) {
        const auto lerp = [&](const int di, const int d) -> double {
            return -half_length[d] + (double)di / (double)(Ns[d] - 1) * full_length[d]; };
        const double z = lerp(zi, 2);
        for (int yi = 0; yi < Ns[1]; ++yi) {
            const double y = lerp(yi, 1);
            for (int xi = 0; xi < Ns[0]; ++xi) {
                const double x = lerp(xi, 0);
                GV.row(xi + Ns[0] * (yi + Ns[1] * zi)) = RowVector3d(x, y, z);
            }
        }
    }

    std::cout << "Marching cubes..." << std::endl;
    MatrixXd SV, BV;
    MatrixXi SF, BF;
    igl::copyleft::marching_cubes(S, GV, Ns[0],Ns[1],Ns[2], SV, SF);
    igl::copyleft::marching_cubes(B, GV, Ns[0],Ns[1],Ns[2], BV, BF);

    std::cout << R"(Usage:
'1'  Show marching cubes contour of signed distance.
'2'  Show marching cubes contour of indicator function.
)";
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(SV, SF);
    viewer.callback_key_down =
        [&](igl::opengl::glfw::Viewer &viewer, unsigned char key, int mod) -> bool {
        switch (key) {
        default:
            return false;
        case '1':
            viewer.data().clear();
            viewer.data().set_mesh(SV, SF);
            break;
        case '2':
            viewer.data().clear();
            viewer.data().set_mesh(BV, BF);
            break;
        }
        viewer.data().set_face_based(true);
        return true;
    };
    viewer.launch();
}
