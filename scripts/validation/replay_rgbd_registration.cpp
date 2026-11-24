// Feed saved Gazebo camera frames to the installed RTAB-Map odometry library.
// No Gazebo reference poses or robot-control interfaces enter this executable.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/SensorData.h>

int main(int argc, char **argv) {
    if (argc < 5 || (argc - 3) % 2 != 0) return 2;
    rtabmap::ParametersMap params;
    const std::string profile(argv[1]);
    if (profile == "distributed" || profile == "octree" || profile == "metric" || profile == "patch") {
        params["Vis/GridRows"] = "2";
        params["Vis/GridCols"] = "2";
        params["GFTT/MinDistance"] = "3";
        if (profile == "metric" || profile == "patch") params["Vis/EstimationType"] = "0";
        if (profile == "patch") {
            params["Odom/Strategy"] = "1";
            params["Vis/CorType"] = "1";
        }
        if (profile == "octree") params["Vis/FeatureType"] = "10";
    } else if (profile != "baseline") return 2;
    auto estimator = std::unique_ptr<rtabmap::Odometry>(rtabmap::Odometry::create(params));
    cv::setNumThreads(1);
    std::ofstream output(argv[2]);
    output << "index,features,matches,inliers,keyframe,valid,x,y,z,roll,pitch,yaw\n";
    for (int arg = 3; arg < argc; arg += 2) {
        auto rgb = cv::imread(argv[arg], cv::IMREAD_COLOR);
        auto depth = cv::imread(argv[arg+1], cv::IMREAD_UNCHANGED);
        if (rgb.empty() || depth.type() != CV_32FC1 || depth.empty()) return 3;
        const char *fov = std::getenv("VIRA_REPLAY_FOV");
        double f = rgb.cols / (2 * std::tan((fov ? std::stod(fov) : 1.15) / 2));
        rtabmap::CameraModel model(f, f, rgb.cols/2., rgb.rows/2.,
                                  rtabmap::CameraModel::opticalRotation(), 0, rgb.size());
        int index = (arg-3)/2;
        rtabmap::SensorData frame(rgb, depth, model, index+1, 1. + index*.1);
        rtabmap::OdometryInfo info;
        auto pose = estimator->process(frame, &info);
        output << index << ',' << info.features << ',' << info.reg.matches << ',' << info.reg.inliers
               << ',' << info.keyFrameAdded << ',' << !pose.isNull();
        if (!pose.isNull()) {
            float x, y, z, r, p, h;
            pose.getTranslationAndEulerAngles(x,y,z,r,p,h);
            output << ',' << x << ',' << y << ',' << z << ',' << r << ',' << p << ',' << h;
        }
        output << '\n';
    }
    return !output.good();
}
