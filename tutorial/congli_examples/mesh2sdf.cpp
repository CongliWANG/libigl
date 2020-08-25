#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>

#include "tutorial_shared_path.h"

int main(int argc, char *argv[]) {
    using namespace Eigen;
    using namespace igl;

    int min_argc = 5;
    std::string mesh_string, sdf_string;
    RowVector3i res;
    if (argc < min_argc) {
        std::cout << "Error: No enough inputs." << std::endl;
        std::cout << "Usage: mesh2sdf <mesh_file> <sdf_file.txt> <N1> <N2> <N3>" << std::endl;
        std::cout << "Running an example ..." << std::endl;
        mesh_string = TUTORIAL_SHARED_PATH "/armadillo.obj";
        sdf_string = "sdf.txt";
        res = RowVector3i(60,70,80);
    } else {
        mesh_string = argv[1];
        sdf_string = argv[2];
        res = RowVector3i(std::stoi(argv[3]),std::stoi(argv[4]),std::stoi(argv[5]));
    }
    
    // Read in inputs as double precision floating point meshes
    MatrixXi F;
    MatrixXd V;
    read_triangle_mesh(mesh_string, V, F);

    // number of vertices on the largest side
    const RowVector3d Vmin = V.colwise().minCoeff();
    const RowVector3d Vmax = V.colwise().maxCoeff();
    const RowVector3d v((Vmax[0]-Vmin[0])/res[0],
                        (Vmax[1]-Vmin[1])/res[1], (Vmax[2]-Vmin[2])/res[2]);
    std::cout << "Res = " << res << std::endl;    
    std::cout << "Voxel size = " << v << std::endl;
    
    // create grid
    std::cout << "Creating grid..." << std::endl;
    MatrixXd GV(res(0) * res(1) * res(2), 3);
    for (int zi = 0; zi < res(2); zi++) {
        const auto lerp = [&](const int di, const int d) -> double {
            return Vmin(d) + (double)di / (double)(res(d) - 1) * (Vmax(d) - Vmin(d)); };
        const double z = lerp(zi, 2);
        for (int yi = 0; yi < res(1); yi++) {
            const double y = lerp(yi, 1);
            for (int xi = 0; xi < res(0); xi++) {
                const double x = lerp(xi, 0);
                GV.row(xi + res(0) * (yi + res(1) * zi)) = RowVector3d(x, y, z);
            }
        }
    }

    // compute values
    std::cout << "Computing distances for ..." << mesh_string << std::endl;
    VectorXd S; //, B;
    VectorXi I;
    MatrixXd C, N;
    signed_distance(GV, V, F, SIGNED_DISTANCE_TYPE_PSEUDONORMAL, S, I, C, N);
    // // Convert distances to binary inside-outside data --> aliasing artifacts
    // B = S;
    // for_each(B.data(), B.data() + B.size(), [](double &b) { b = (b > 0 ? 1 : (b < 0 ? -1 : 0)); });

    // save distance field to .txt (line-by-line)
    std::ofstream file(sdf_string);
    if (file.is_open()) {
        // first line: N1 N2 N3 v1 v2 v3
        file << res[0] << " " << res[1] << " " << res[2] << " "
             << v[0] << " " << v[1] << " " << v[2] << std::endl;
        for (int i = 0; i < S.size(); ++i)
            file << S[i] << std::endl; //std::setprecision(std::numeric_limits<float>::digits10 + 1) <<
        file.close();
    }
    std::cout << "Distance field saved as " << sdf_string << "." << std::endl;
}
