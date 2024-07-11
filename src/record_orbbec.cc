#include <popl.hpp>
#include <ghc/filesystem.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = ghc::filesystem;

int main(int argc, char* argv[]) {
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto device_index = op.add<popl::Value<unsigned int>>("d", "device", "device index", 0);
    auto output_path = op.add<popl::Value<std::string>>("o", "output", "output folder path");
    auto force = op.add<popl::Switch>("f", "force", "force overwrite output folder");
    auto color_width = op.add<popl::Value<unsigned int>>("", "color-width", "color image width", 1280);
    auto color_height = op.add<popl::Value<unsigned int>>("", "color-height", "color image height", 960);
    auto color_fps = op.add<popl::Value<unsigned int>>("", "color-fps", "color image fps", 30);
    auto depth_width = op.add<popl::Value<unsigned int>>("", "depth-width", "depth image width", 640);
    auto depth_height = op.add<popl::Value<unsigned int>>("", "depth-height", "depth image height", 576);
    auto depth_fps = op.add<popl::Value<unsigned int>>("", "depth-fps", "depth image fps", 30);
    auto depth_threshold = op.add<popl::Value<float>>("", "depth-threshold", "depth threshold", 40.0);

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!op.unknown_options().empty()) {
        for (const auto& unknown_option : op.unknown_options()) {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!output_path->is_set()) {
        std::cerr << "Error: please provide an output path" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // Create output directories
    const auto config_path = fs::path(output_path->value()) / "config.yaml";

    // check if output directory exists
    if (fs::exists(config_path)) {
        if (!force->is_set()) {
            std::cerr << "Error: output directory already exists" << std::endl;
            return -1;
        }

        fs::remove_all(config_path.parent_path());
    }

    fs::create_directories(config_path.parent_path());
    fs::create_directories(fs::path(output_path->value()) / "rgb");
    fs::create_directories(fs::path(output_path->value()) / "depth");

    // Open the device
    ob::Context ctx;
    auto devList = ctx.queryDeviceList();

    if(devList->deviceCount() <= device_index->value()) {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    auto dev = devList->getDevice(device_index->value());

    auto devConfig = std::make_shared<ob::Config>();
    devConfig->enableVideoStream(OB_STREAM_COLOR, color_width->value(), color_height->value(), color_fps->value(), OB_FORMAT_MJPG);
    devConfig->enableVideoStream(OB_STREAM_DEPTH, depth_width->value(), depth_height->value(), depth_fps->value(), OB_FORMAT_Y16);
    // devConfig->enableAccelStream();
    devConfig->setAlignMode(ALIGN_D2C_SW_MODE);

    ob::Pipeline pipe;
    pipe.enableFrameSync();
    pipe.start(devConfig);

    // Get depth range based on depth resolution
    float depth_range_max;

    switch (depth_width->value()) {
        default:
            depth_range_max = 2.21;
            break;
        case 512:
            depth_range_max = 2.88;
            break;
        case 640:
            depth_range_max = 3.86;
            break;
        case 320:
            depth_range_max = 5.46;
            break;
    }

    // write vslam config
    auto params = pipe.getCameraParam();
    fs::ofstream config_file(config_path);
    config_file << "Camera:" << std::endl;
    config_file << "  name: \"Orbbec\"" << std::endl;
    config_file << "  setup: \"RGBD\"" << std::endl;
    config_file << "  model: \"perspective\"" << std::endl;
    config_file << std::endl;
    config_file << "  fx: " << params.rgbIntrinsic.fx << std::endl;
    config_file << "  fy: " << params.rgbIntrinsic.fy << std::endl;
    config_file << "  cx: " << params.rgbIntrinsic.cx << std::endl;
    config_file << "  cy: " << params.rgbIntrinsic.cy << std::endl;
    config_file << std::endl;
    config_file << "  k1: " << params.rgbDistortion.k1 << std::endl;
    config_file << "  k2: " << params.rgbDistortion.k2 << std::endl;
    config_file << "  p1: " << params.rgbDistortion.p1 << std::endl;
    config_file << "  p2: " << params.rgbDistortion.p2 << std::endl;
    config_file << "  k3: " << params.rgbDistortion.k3 << std::endl;
    config_file << std::endl;
    config_file << "  fps: " << color_fps->value() << std::endl;
    config_file << "  cols: " << color_width->value() << std::endl;
    config_file << "  rows: " << color_height->value() << std::endl;
    config_file << "  color_order: \"RGB\"" << std::endl;
    config_file << std::endl;
    config_file << "  focal_x_baseline: " << (depth_range_max/depth_threshold->value()*params.rgbIntrinsic.fx) << std::endl;
    config_file << "  depth_threshold: " << depth_threshold->value() << std::endl;
    config_file << std::endl;
    config_file << "Preprocessing:" << std::endl;
    config_file << "  min_size: 800" << std::endl;
    config_file << "  depthmap_factor: 1000.0" << std::endl;
    config_file << std::endl;
    config_file.close();

    // Initialize txt files
    fs::ofstream rgb_txt(fs::path(output_path->value()) / "rgb.txt");
    fs::ofstream depth_txt(fs::path(output_path->value()) / "depth.txt");
    // fs::ofstream accelerometer_txt(fs::path(output_path->value()) / "accelerometer.txt");

    rgb_txt << "# color images" << std::endl;
    rgb_txt << "# timestamp filename" << std::endl;

    depth_txt << "# depth maps" << std::endl;
    depth_txt << "# timestamp filename" << std::endl;
    
    // accelerometer_txt << "# accelerometer data" << std::endl;
    // accelerometer_txt << "# timestamp ax ay az" << std::endl;

    // Start recording until keyboard interrupt
    std::cout << "Recording..." << std::endl;

    cv::Mat depth_mat(color_height->value(), color_width->value(), CV_16UC1);

    while (true) {
        auto frame_set = pipe.waitForFrames(100);

        if (frame_set == nullptr) {
            continue;
        }

        // Get timestamp
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
        auto timestamp_str = std::to_string(timestamp);

        // Write color as jpeg
        auto color_frame = frame_set->colorFrame();

        if (color_frame != nullptr) {
            const auto color_path = fs::path(output_path->value()) / "rgb" / (timestamp_str + ".jpg");
            fs::ofstream color_file(color_path, std::ios::binary);
            color_file.write(reinterpret_cast<const char*>(color_frame->data()), color_frame->dataSize());
            color_file.close();

            rgb_txt << timestamp_str << " rgb/" << color_path.filename().string() << std::endl;
        }

        // Write depth as png
        auto depth_frame = frame_set->depthFrame();

        if (depth_frame != nullptr) {
            memcpy(depth_mat.data, depth_frame->data(), depth_frame->dataSize());
            const auto depth_path = fs::path(output_path->value()) / "depth" / (timestamp_str + ".png");
            cv::imwrite(depth_path, depth_mat);

            depth_txt << timestamp_str << " depth/" << depth_path.filename().string() << std::endl;
        }
    }

    // Stop
    pipe.stop();

    // Close txt files
    rgb_txt.close();
    depth_txt.close();
    // accelerometer_txt.close();

    return 0;
}