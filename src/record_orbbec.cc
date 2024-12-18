#include <signal.h>

#include <popl.hpp>
#include <ghc/filesystem.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <condition_variable>

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
    auto color_size = op.add<popl::Value<std::string>>("", "color-size", "color image resolution (hd, 960p, fhd, qhd, 4k)", "hd");
    auto color_exposure = op.add<popl::Value<unsigned int>>("", "color-exposure", "color exposure");
    auto color_gain = op.add<popl::Value<unsigned int>>("", "color-gain", "color gain", 0);
    auto color_format = op.add<popl::Value<std::string>>("", "color-format", "color image format (jpg, png)", "jpg");
    auto white_balance = op.add<popl::Value<unsigned int>>("", "white-balance", "white balance");
    auto depth_size = op.add<popl::Value<std::string>>("", "depth-size", "depth image resolution (288p, 512p, 576p, 1024p)", "576p");
    auto fps = op.add<popl::Value<unsigned int>>("", "fps", "fps", 30);
    auto depth_threshold = op.add<popl::Value<float>>("", "depth-threshold", "depth threshold", 40.0);
    auto viewer = op.add<popl::Switch>("", "viewer", "enable viewer");
    auto wait = op.add<popl::Value<unsigned int>>("", "wait", "wait time in seconds for auto exposure", 5);

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

    // Get color resolution
    unsigned int color_width, color_height;

    if (color_size->value() == "hd")
    {
        color_width = 1280;
        color_height = 720;
    }
    else if (color_size->value() == "960p")
    {
        color_width = 1280;
        color_height = 960;
    }
    else if (color_size->value() == "fhd")
    {
        color_width = 1920;
        color_height = 1080;
    }
    else if (color_size->value() == "qhd")
    {
        color_width = 2560;
        color_height = 1440;
    }
    else if (color_size->value() == "4k")
    {
        color_width = 3840;
        color_height = 2160;
    }
    else
    {
        std::cerr << "Error: invalid color size, must be hd, 960p, fhd, qhd, 4k" << std::endl;
        return EXIT_FAILURE;
    }

    // Get depth resolution
    unsigned int depth_width, depth_height;
    float depth_range_max;

    if (depth_size->value() == "288p") // binned
    {
        depth_width = 320;
        depth_height = 288;
        depth_range_max = 5.46;
    }
    else if (depth_size->value() == "512p") // binned
    {
        depth_width = 512;
        depth_height = 512;
        depth_range_max = 2.88;
    }
    else if (depth_size->value() == "576p") // unbinned
    {
        depth_width = 640;
        depth_height = 576;
        depth_range_max = 3.86;
    }
    else if (depth_size->value() == "1024p") // unbinned
    {
        depth_width = 1024;
        depth_height = 1024;
        depth_range_max = 2.21;
    }
    else
    {
        std::cerr << "Error: invalid depth size, must be 288p, 512p, 576p, 1024p" << std::endl;
        return EXIT_FAILURE;
    }

    // Get color format
    OBFormat ob_color_format;

    if (color_format->value() == "jpg")
    {
        ob_color_format = OB_FORMAT_MJPG;
    }
    else if (color_format->value() == "png")
    {
        ob_color_format = OB_FORMAT_RGB;
    }
    else
    {
        std::cerr << "Error: invalid color format, must be jpg or png" << std::endl;
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
    fs::create_directories(fs::path(output_path->value()) / "ir");

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
    devConfig->enableVideoStream(OB_STREAM_COLOR, color_width, color_height, fps->value(), OB_FORMAT_RGB888);
    devConfig->enableVideoStream(OB_STREAM_DEPTH, depth_width, depth_height, fps->value(), OB_FORMAT_Y16);
    devConfig->enableVideoStream(OB_STREAM_IR, depth_width, depth_height, fps->value(), OB_FORMAT_Y16);
    // devConfig->setAlignMode(ALIGN_D2C_SW_MODE);

    // apply exposure settings
    dev->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, !color_exposure->is_set());

    if (color_exposure->is_set())
    {
        dev->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, color_exposure->value());
    }

    // apply white balance settings
    dev->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, !white_balance->is_set());

    if (white_balance->is_set())
    {
        dev->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, white_balance->value());
    }

    // set gain
    dev->setIntProperty(OB_PROP_COLOR_GAIN_INT, color_gain->value());

    // Start the pipeline
    ob::Pipeline pipe;
    pipe.enableFrameSync();
    pipe.start(devConfig);

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
    config_file << "  fps: " << fps->value() << "\n";
    config_file << "  cols: " << color_width << "\n";
    config_file << "  rows: " << color_height << "\n";
    config_file << "  color_order: \"RGB\"\n";
    config_file << "\n";
    config_file << "  focal_x_baseline: " << (depth_range_max / depth_threshold->value() * params.rgbIntrinsic.fx) << "\n";
    config_file << "  depth_threshold: " << depth_threshold->value() << "\n";
    config_file << "\n";

    config_file << "DepthCamera:\n";
    config_file << "  fx: " << params.depthIntrinsic.fx << "\n";
    config_file << "  fy: " << params.depthIntrinsic.fy << "\n";
    config_file << "  cx: " << params.depthIntrinsic.cx << "\n";
    config_file << "  cy: " << params.depthIntrinsic.cy << "\n";
    config_file << "\n";
    config_file << "  k1: " << params.depthDistortion.k1 << "\n";
    config_file << "  k2: " << params.depthDistortion.k2 << "\n";
    config_file << "  p1: " << params.depthDistortion.p1 << "\n";
    config_file << "  p2: " << params.depthDistortion.p2 << "\n";
    config_file << "  k3: " << params.depthDistortion.k3 << "\n";
    config_file << "\n";
    config_file << "  cols: " << depth_width << "\n";
    config_file << "  rows: " << depth_height << "\n";
    config_file << "\n";

    config_file << "Preprocessing:\n";
    config_file << "  min_size: 800\n";
    config_file << "  depthmap_factor: 1000.0\n";
    config_file.close();

    // Initialize txt files
    fs::ofstream rgb_txt(fs::path(output_path->value()) / "rgb.txt");
    fs::ofstream depth_txt(fs::path(output_path->value()) / "depth.txt");
    fs::ofstream imu_txt(fs::path(output_path->value()) / "imu.txt");
    fs::ofstream ir_txt(fs::path(output_path->value()) / "ir.txt");

    rgb_txt << "# color images" << std::endl;
    rgb_txt << "# timestamp [s] | filename" << std::endl;

    depth_txt << "# depth maps" << std::endl;
    depth_txt << "# timestamp [s] | filename" << std::endl;

    imu_txt << "# accelerometer and gyroscope data" << std::endl;
    imu_txt << "# timestamp [s] | acc_x [m/s^2] | acc_y [m/s^2] | acc_z [m/s^2] | gyro_x [rad/s] | gyro_y [rad/s] | gyro_z [rad/s]" << std::endl;

    ir_txt << "# ir images" << std::endl;
    ir_txt << "# timestamp [s] | filename" << std::endl;

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

    // // Write to disk on separate thread
    // std::deque<std::pair<std::string, std::vector<uchar>>> write_queue;
    // std::mutex write_mutex;
    // std::condition_variable write_cv;

    // auto write_thread = std::thread([&]()
    //                                 {
    //     while (running)
    //     {
    //         std::pair<std::string, std::vector<uchar>> item;

    //         {
    //             std::unique_lock<std::mutex> lock(write_mutex);
    //             write_cv.wait(lock, [&]()
    //                           { return !write_queue.empty() || !running; });

    //             if (!running)
    //             {
    //                 break;
    //             }

    //             item = write_queue.front();
    //             write_queue.pop_front();
    //         }

    //         // Write to disk
    //         fs::ofstream file(item.first, std::ios::binary);
    //         file.write(reinterpret_cast<const char *>(item.second.data()), item.second.size());
    //         file.close();
    //     } });

    // Compress images on separate thread
    std::deque<std::pair<std::string, cv::Mat>> compress_queue;
    std::mutex compress_mutex;
    std::condition_variable compress_cv;

    auto compress_thread = std::thread([&]()
                                       {
        while (running)
        {
            std::pair<std::string, cv::Mat> item;

            {
                std::unique_lock<std::mutex> lock(compress_mutex);
                compress_cv.wait(lock, [&]()
                                 { return !compress_queue.empty() || !running; });

                if (!running)
                {
                    break;
                }

                item = compress_queue.front();
                compress_queue.pop_front();
            }

            cv::imwrite(item.first, item.second);

            // // Convert to jpg or png and send to write queue
            // auto format = item.first.substr(item.first.find_last_of("."));
            // std::vector<uchar> buf;
            // cv::imencode(format, item.second, buf);

            // {
            //     std::lock_guard<std::mutex> lock(write_mutex);
            //     write_queue.push_back({item.first, buf});
            //     write_cv.notify_one();
            // }
        } });

    // Record color and depth frames
    auto start_time = std::chrono::high_resolution_clock::now();

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

        // Queue items
        std::vector<std::pair<std::string, cv::Mat>> queue_items;

        // Write color as jpeg
        auto color_frame = frame_set->colorFrame();

        if (color_frame != nullptr)
        {
            auto timestamp = color_frame->timeStamp();
            auto timestamp_str = std::to_string(timestamp / 1e3);
            const auto color_path = fs::path(output_path->value()) / "rgb" / (timestamp_str + "." + color_format->value());

            // Write as jpg or png
            cv::Mat color_mat(color_frame->height(), color_frame->width(), CV_8UC3, color_frame->data());
            cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
            // cv::imwrite(color_path, color_mat);
            queue_items.push_back({color_path, color_mat});

            // Write to txt file
            rgb_txt << timestamp_str << " rgb/" << color_path.filename().string() << std::endl;

            // View color image
            if (viewer->is_set())
            {
                cv::imshow("Color", color_mat.clone());
                cv::waitKey(1);
            }
        }

        // Write depth as png
        auto depth_frame = frame_set->depthFrame();

        if (depth_frame != nullptr)
        {
            auto timestamp = depth_frame->timeStamp();
            auto timestamp_str = std::to_string(timestamp / 1e3);

            // Write as png
            cv::Mat depth_mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data());
            const auto depth_path = fs::path(output_path->value()) / "depth" / (timestamp_str + ".png");
            // cv::imwrite(depth_path, depth_mat);
            queue_items.push_back({depth_path, depth_mat});

            // Write to txt file
            depth_txt << timestamp_str << " depth/" << depth_path.filename().string() << std::endl;

            // View depth image
            if (viewer->is_set())
            {
                cv::Mat depth_mat_8u;
                cv::normalize(depth_mat, depth_mat_8u, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                cv::imshow("Depth", depth_mat_8u);
                cv::waitKey(1);
            }
        }

        // Write ir as png
        auto ir_frame = frame_set->irFrame();

        if (ir_frame != nullptr)
        {
            auto timestamp = ir_frame->timeStamp();
            auto timestamp_str = std::to_string(timestamp / 1e3);

            // Write as png
            cv::Mat ir_mat(ir_frame->height(), ir_frame->width(), CV_16UC1, ir_frame->data());
            const auto ir_path = fs::path(output_path->value()) / "ir" / (timestamp_str + ".png");
            // cv::imwrite(ir_path, ir_mat);
            queue_items.push_back({ir_path, ir_mat});

            // Write to txt file
            ir_txt << timestamp_str << " ir/" << ir_path.filename().string() << std::endl;

            // View ir image
            if (viewer->is_set())
            {
                cv::Mat ir_mat_8u;
                cv::normalize(ir_mat, ir_mat_8u, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                cv::imshow("IR", ir_mat_8u);
                cv::waitKey(1);
            }
        }

        // Queue items for compression if queue is empty and wait time has passed
        auto current_time = std::chrono::high_resolution_clock::now();

        if ((current_time - start_time) > std::chrono::seconds(wait->value()))
        {
            std::lock_guard<std::mutex> lock(compress_mutex);

            if (compress_queue.empty())
            {
                compress_queue.insert(compress_queue.end(), queue_items.begin(), queue_items.end());
                compress_cv.notify_one();
            }
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

    // Close opencv windows
    cv::destroyAllWindows();

    return 0;
}
