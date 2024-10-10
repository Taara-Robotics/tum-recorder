#include <signal.h>

#include <popl.hpp>
#include <ghc/filesystem.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = ghc::filesystem;

bool running = true;

void interrupt_handler(int)
{
    std::cout << "Interrupted" << std::endl;
    running = false;
}

int main(int argc, char *argv[])
{
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto device_index = op.add<popl::Value<unsigned int>>("d", "device", "device index", 0);
    auto output_path = op.add<popl::Value<std::string>>("o", "output", "output folder path");
    auto force = op.add<popl::Switch>("f", "force", "force overwrite output folder");
    auto color_width = op.add<popl::Value<unsigned int>>("", "color-width", "color image width", 1280);
    auto color_height = op.add<popl::Value<unsigned int>>("", "color-height", "color image height", 960);
    auto color_fps = op.add<popl::Value<unsigned int>>("", "color-fps", "color image fps", 30);
    auto color_exposure = op.add<popl::Value<unsigned int>>("", "color-exposure", "color exposure", 100);
    auto white_balance = op.add<popl::Value<unsigned int>>("", "white-balance", "white balance", 4000);
    auto depth_width = op.add<popl::Value<unsigned int>>("", "depth-width", "depth image width", 640);
    auto depth_height = op.add<popl::Value<unsigned int>>("", "depth-height", "depth image height", 576);
    auto depth_fps = op.add<popl::Value<unsigned int>>("", "depth-fps", "depth image fps", 30);
    auto depth_threshold = op.add<popl::Value<float>>("", "depth-threshold", "depth threshold", 40.0);

    try
    {
        op.parse(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set())
    {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!op.unknown_options().empty())
    {
        for (const auto &unknown_option : op.unknown_options())
        {
            std::cerr << "unknown_options: " << unknown_option << std::endl;
        }
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!output_path->is_set())
    {
        std::cerr << "Error: please provide an output path" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // Register interrupt handler
    struct sigaction sig_int_handler;
    sig_int_handler.sa_handler = interrupt_handler;
    sigemptyset(&sig_int_handler.sa_mask);
    sig_int_handler.sa_flags = 0;
    sigaction(SIGINT, &sig_int_handler, NULL);

    // Create output directories
    const auto config_path = fs::path(output_path->value()) / "config.yaml";

    // check if output directory exists
    if (fs::exists(config_path))
    {
        if (!force->is_set())
        {
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

    if (devList->deviceCount() <= device_index->value())
    {
        std::cerr << "Device not found!" << std::endl;
        return -1;
    }

    auto dev = devList->getDevice(device_index->value());

    auto devConfig = std::make_shared<ob::Config>();
    devConfig->enableVideoStream(OB_STREAM_COLOR, color_width->value(), color_height->value(), color_fps->value(), OB_FORMAT_MJPG);
    devConfig->enableVideoStream(OB_STREAM_DEPTH, depth_width->value(), depth_height->value(), depth_fps->value(), OB_FORMAT_Y16);
    devConfig->setAlignMode(ALIGN_D2C_SW_MODE);

    // disable auto white balance
    dev->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, false);
    dev->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, white_balance->value());

    // disable auto exposure
    dev->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
    dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, color_exposure->value());

    // Start the pipeline
    ob::Pipeline pipe;
    pipe.enableFrameSync();
    pipe.start(devConfig);

    // Get depth range based on depth resolution
    float depth_range_max;

    switch (depth_width->value())
    {
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
    config_file << "Camera:\n";
    config_file << "  name: \"Orbbec\"\n";
    config_file << "  setup: \"RGBD\"\n";
    config_file << "  model: \"perspective\"\n";
    config_file << "\n";
    config_file << "  fx: " << params.rgbIntrinsic.fx << "\n";
    config_file << "  fy: " << params.rgbIntrinsic.fy << "\n";
    config_file << "  cx: " << params.rgbIntrinsic.cx << "\n";
    config_file << "  cy: " << params.rgbIntrinsic.cy << "\n";
    config_file << "\n";
    config_file << "  k1: " << params.rgbDistortion.k1 << "\n";
    config_file << "  k2: " << params.rgbDistortion.k2 << "\n";
    config_file << "  p1: " << params.rgbDistortion.p1 << "\n";
    config_file << "  p2: " << params.rgbDistortion.p2 << "\n";
    config_file << "  k3: " << params.rgbDistortion.k3 << "\n";
    config_file << "\n";
    config_file << "  fps: " << color_fps->value() << "\n";
    config_file << "  cols: " << color_width->value() << "\n";
    config_file << "  rows: " << color_height->value() << "\n";
    config_file << "  color_order: \"RGB\"\n";
    config_file << "\n";
    config_file << "  focal_x_baseline: " << (depth_range_max / depth_threshold->value() * params.rgbIntrinsic.fx) << "\n";
    config_file << "  depth_threshold: " << depth_threshold->value() << "\n";
    config_file << "\n";
    config_file << "Preprocessing:\n";
    config_file << "  min_size: 800\n";
    config_file << "  depthmap_factor: 1000.0\n";
    config_file.close();

    // Initialize txt files
    fs::ofstream rgb_txt(fs::path(output_path->value()) / "rgb.txt");
    fs::ofstream depth_txt(fs::path(output_path->value()) / "depth.txt");
    fs::ofstream imu_txt(fs::path(output_path->value()) / "imu.txt");

    rgb_txt << "# color images" << std::endl;
    rgb_txt << "# timestamp [s] | filename" << std::endl;

    depth_txt << "# depth maps" << std::endl;
    depth_txt << "# timestamp [s] | filename" << std::endl;

    imu_txt << "# accelerometer and gyroscope data" << std::endl;
    imu_txt << "# timestamp [s] | acc_x [m/s^2] | acc_y [m/s^2] | acc_z [m/s^2] | gyro_x [rad/s] | gyro_y [rad/s] | gyro_z [rad/s]" << std::endl;

    // Queue accelerometer and gyroscope data to combine by timestamps in main thread
    std::mutex imu_mutex;
    std::map<uint64_t, OBAccelValue> accel_data;
    std::map<uint64_t, OBGyroValue> gyro_data;

    // Record accelerometer
    auto accel_sensor = dev->getSensorList()->getSensor(OB_SENSOR_ACCEL);

    if (accel_sensor != nullptr)
    {
        auto accel_profiles = accel_sensor->getStreamProfileList();
        auto accel_profile = accel_profiles->getProfile(OB_PROFILE_DEFAULT);

        accel_sensor->start(accel_profile, [&imu_mutex, &accel_data](std::shared_ptr<ob::Frame> frame)
                            {
            auto timestamp = frame->timeStamp();
            auto accel_frame = frame->as<ob::AccelFrame>();

            // Add to accel data map using the mutex
            if (accel_frame != nullptr) {
                std::lock_guard<std::mutex> lock(imu_mutex);
                accel_data[timestamp] = accel_frame->value();
            } });
    }

    // Record gyroscope
    auto gyro_sensor = dev->getSensorList()->getSensor(OB_SENSOR_GYRO);

    if (gyro_sensor)
    {
        auto gyro_profiles = gyro_sensor->getStreamProfileList();
        auto gyro_profile = gyro_profiles->getProfile(OB_PROFILE_DEFAULT);

        gyro_sensor->start(gyro_profile, [&imu_mutex, &gyro_data](std::shared_ptr<ob::Frame> frame)
                           {
            auto timestamp = frame->timeStamp();
            auto gyro_frame = frame->as<ob::GyroFrame>();

            if (gyro_frame != nullptr) {
                std::lock_guard<std::mutex> lock(imu_mutex);
                gyro_data[timestamp] = gyro_frame->value();
            } });
    }

    // Record color and depth frames
    cv::Mat depth_mat(color_height->value(), color_width->value(), CV_16UC1);

    while (running)
    {
        auto frame_set = pipe.waitForFrames(100);

        if (frame_set == nullptr)
        {
            continue;
        }

        // Combine accelerometer and gyroscope data
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            std::vector<uint64_t> timestamps_to_remove;

            for (auto &[timestamp, accel_value] : accel_data)
            {
                auto it = gyro_data.find(timestamp);

                if (it != gyro_data.end())
                {
                    auto gyro_value = it->second;
                    auto timestamp_str = std::to_string(timestamp / 1e3);
                    imu_txt << timestamp_str << " " << accel_value.x << " " << accel_value.y << " " << accel_value.z << " " << gyro_value.x << " " << gyro_value.y << " " << gyro_value.z << std::endl;
                    timestamps_to_remove.push_back(timestamp);
                }
            }

            // Clear combined data
            for (auto timestamp : timestamps_to_remove)
            {
                accel_data.erase(timestamp);
                gyro_data.erase(timestamp);
            }
        }

        // Write color as jpeg
        auto color_frame = frame_set->colorFrame();

        if (color_frame != nullptr)
        {
            auto timestamp = color_frame->timeStamp();
            auto timestamp_str = std::to_string(timestamp / 1e3);
            const auto color_path = fs::path(output_path->value()) / "rgb" / (timestamp_str + ".jpg");
            fs::ofstream color_file(color_path, std::ios::binary);
            color_file.write(reinterpret_cast<const char *>(color_frame->data()), color_frame->dataSize());
            color_file.close();

            rgb_txt << timestamp_str << " rgb/" << color_path.filename().string() << std::endl;
        }

        // Write depth as png
        auto depth_frame = frame_set->depthFrame();

        if (depth_frame != nullptr)
        {
            auto timestamp = depth_frame->timeStamp();
            auto timestamp_str = std::to_string(timestamp / 1e3);
            memcpy(depth_mat.data, depth_frame->data(), depth_frame->dataSize());
            const auto depth_path = fs::path(output_path->value()) / "depth" / (timestamp_str + ".png");
            cv::imwrite(depth_path, depth_mat);

            depth_txt << timestamp_str << " depth/" << depth_path.filename().string() << std::endl;
        }
    }

    // Stop
    if (accel_sensor != nullptr)
    {
        accel_sensor->stop();
    }

    if (gyro_sensor != nullptr)
    {
        gyro_sensor->stop();
    }

    pipe.stop();

    // Close txt files
    rgb_txt.close();
    depth_txt.close();
    imu_txt.close();

    return 0;
}
