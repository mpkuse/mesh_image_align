#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

#include <theia/theia.h>

#include "utils/RawFileIO.h"
#include "utils/CameraGeometry.h"

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"


using namespace std;

void get_me_a_pose( Matrix4d& c_T_w )
{
    PoseManipUtils::rawyprt_to_eigenmat( Vector3d(0,0,0), Vector3d(0,0,-10.), c_T_w );

    MatrixXd w_X = MatrixXd::Zero( 4, 5 );
    MatrixXd uv = MatrixXd::Zero( 3, 5 );

    w_X.col(0) << 0.15984, -1.0127, 0., 1.0;
    w_X.col(1) << 4.6385, 4.3482, -0.1962, 1.0;
    w_X.col(2) << 5.039, 1.9453, -0.4106, 1.0;
    w_X.col(3) << 9.2532, 1.0152, 0.03678, 1.0;
    w_X.col(4) << 10.1126, 0.18865, 0., 1.0;

    uv.col(0) << 296, 405, 1.;
    uv.col(1) << 330, 306, 1.;
    uv.col(2) << 364, 337, 1.;
    uv.col(3) << 443, 334, 1.;
    uv.col(4) << 484, 348, 1.;

    Matrix3d K_new;
    GeometryUtils::make_K( 375.0, 375.0, 376.0, 240.0, K_new ); // this can be any arbitrary values, but usually you want to have fx,fy in a similar range to original intrinsic and cx,cy as half of image width and height respectively so that you can view the image correctly
    MatrixXd uv_normalized = K_new.inverse() * uv;

    cout << "w_X\n" << w_X << endl;
    cout << "uv\n" << uv << endl;

    vector<Vector2d> x;//[5]; //image pts
    vector<Vector3d> X;//[5]; //world pts
    for( int i=0; i<5 ; i++ ) {
        x.push_back( uv_normalized.col(i).topRows(2) ); //Vector2d::Random();
        X.push_back( w_X.col(i).topRows(3) ); //Vector3d::Random();
    }
    std::vector<Eigen::Quaterniond> solution_rotations;
    std::vector<Eigen::Vector3d> solution_translations;
    theia::DlsPnp( x, X, &solution_rotations, &solution_translations );
    std::cout << "#solutions " << solution_translations.size() << " " << solution_rotations.size();

    Matrix4d sol = Matrix4d::Identity();
    sol.topLeftCorner(3,3) = solution_rotations[0].toRotationMatrix();
    sol.col(3).topRows(3) = solution_translations[0];
    c_T_w = sol; //sol.inverse();
    cout << PoseManipUtils::prettyprintMatrix4d( c_T_w ) << endl;

}


int main( int argc, char ** argv )
{
    // Load Image
    // cv::Mat im = cv::imread( "../data/left/1544776507771075010.jpg");
    cv::Mat im = cv::imread( "../data/demo_image.jpg");
    cv::imshow( "raw win", im );
    // cv::waitKey(0);


    MatrixXd X;
    RawFileIO::read_eigen_matrix( "../data/banana.xyz", X );
    std::cout << X.rows() << "," << X.cols() << endl;
    // cout << X << endl;
    MatrixXd X_h = MatrixXd::Constant( 4, X.rows(), 1.0 );
    X_h.topRows(3) = X.transpose();
    cout << X_h.leftCols(10) << endl;


    // camera
    std::string calib_file = "../data/left.yaml";
    camodocal::CameraPtr m_camera;
    std::cout << ((m_camera)?"cam is initialized":"cam is not initiazed") << std::endl; //this should print 'not initialized'
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    std::cout << ((m_camera)?"cam is initialized":"cam is not initiazed") << std::endl; //this should print 'initialized'
    std::cout << m_camera->parametersToString() << std::endl;
    std::cout << "imageWidth="  << m_camera->imageWidth() << std::endl;
    std::cout << "imageHeight=" << m_camera->imageHeight() << std::endl;
    std::cout << "model_type=" << m_camera->modelType() << std::endl;
    std::cout << "camera_name=" << m_camera->cameraName() << std::endl;


    // try undistort
    Matrix3d K_new;
    GeometryUtils::make_K( 375.0, 375.0, 376.0, 240.0, K_new ); // this can be any arbitrary values, but usually you want to have fx,fy in a similar range to original intrinsic and cx,cy as half of image width and height respectively so that you can view the image correctly
    cout << "K_new="<< K_new << endl;

    // MonoGeometry monogeom( m_camera );
    std::shared_ptr<MonoGeometry> monogeom;
    monogeom = std::make_shared<MonoGeometry>( m_camera  );
    monogeom->set_K( K_new );

    cv::Mat im_undistorted;
    monogeom->do_image_undistortion( im, im_undistorted );


    MatrixXd uv;
    Matrix4d c_T_w;
    get_me_a_pose( c_T_w );

    GeometryUtils::idealProjection( K_new, c_T_w*X_h, uv );
    // cout << "uv:\n" << uv << endl;
    MiscUtils::plot_point_sets( im_undistorted, uv, cv::Scalar(0,0,255), false, ""  );

    cv::imshow( "undistort", im_undistorted );
    cv::waitKey(0);



    cout << "Hello World\n";
    return 0;
}
