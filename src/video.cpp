#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include "elevation.h"
#include "map_generator.h"
#include "catenate_video.h"
#include "configuration.h"
#include "telemetry.h"
#include "layout.h"

namespace fs = std::filesystem;

static Layout read_layout(fs::path path, const Configuration &conf, CatenateVideo &catv, cv::Mat &telemetry, std::map<int, MapGenerator> &map_generators)
{
    Layout retval;
    
    while (true)
    {
        try
        {
            retval = Layout{path};

            for (auto *image_generator : retval.internal_images("map"))
            {
                std::cout << "Create map" << std::endl;
                if (image_generator->parameters_.size() != 1)
                    throw std::runtime_error("One and only argument is expected for map : zoom");
                int zoom = std::stoi(image_generator->parameters_.front());
                int size = image_generator->size_.width;

                auto &map_generator = map_generators.try_emplace(size, conf.osm_xml_, conf.osm_cache_, size).first->second;
                image_generator->generator_ = 
                    [&map_generator, &telemetry, zoom](int time)->cv::Mat
                    {
                        time = std::min(time, telemetry.cols - 1);
                        return map_generator(
                            telemetry.at<double>(Telemetry::LATITUDE, time),
                            telemetry.at<double>(Telemetry::LONGITUDE, time),
                            zoom);
                    };
            }

            for (auto *image_generator : retval.internal_images("gopro"))
            {
                std::cout << "Create video" << std::endl;
                image_generator->generator_ = 
                    [&catv](int time)->cv::Mat
                    {
                        return catv.fetch_frame(time);
                    };
            }

            break;
        }
        catch (std::exception &e)
        {
            std::cout << "Impossible to read " << path << ". Reason : " << e.what() << std::endl;
            std::cout << "Please modify the file and try again. Press Enter to try again" << std::endl;
            std::cin.ignore();
        }
    }
    return retval;
}

int main()
{
    Configuration conf("data.json"); 

    Elevation::init();
    MapGenerator::init();

    std::cout << "Go" << std::endl;

    std::map<int, MapGenerator> map_generators;

    std::cout << "Read telemetry" << std::endl;
    auto telemetry = Telemetry{conf}.data();
  
    std::cout << "Read video" << std::endl;
    CatenateVideo catv(conf);

    bool fast = false;
    bool pause = true;

    std::cout << "Read layout" << std::endl;
    Layout layout = read_layout(conf.layout_json_, conf, catv, telemetry, map_generators);

    std::cout << "play! " << layout.output_bounds_ << std::endl;

    int frame_position = 0;
    while (true)
    {
        auto output = layout.render(frame_position, {{"gopro", 45}});

        cv::imshow("test", output);
        switch (cv::waitKey(1))
        {
            case 'q':
                return 0;
            case 'f':
                fast = !fast;
                catv.fast(fast);
                break;
            case 'l':
                frame_position += catv.fps() * catv.step() * 10;
                frame_position = std::min(catv.count(), frame_position);
                break;
            case 'j':
                frame_position -= catv.fps() * catv.step() * 10;
                frame_position = std::max(0, frame_position);
                break;
            case 'k':
                pause = !pause;
                break;
            case 'r':
                layout = read_layout(conf.layout_json_, conf, catv, telemetry, map_generators);
                break;
            default:
                frame_position += catv.step() * !pause;
                break;
        }
    }
    return {};


    // cv::VideoCapture video("/home/phi/dev/slowtv/video/concat.mp4");
    // std::cout << video.get(cv::CAP_PROP_FRAME_COUNT) << std::endl;
    // cv::Mat img;

    // video.set(cv::CAP_PROP_POS_MSEC, 3600000);

    // // for (int i = 0; i != 216000; i++)
    // // {
    // //     video.read(img);
    // //     if (i % 60 == 0)
    // //         std::cout << (i / 3600) << ":" << (i / 60) % 60 << std::endl;
    // // }


    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // for (int i = 0; i != 120; i++)
    // {
    //     auto map = image_generator(60.1699,24.9384);
    //     video.read(img);
    //     map.copyTo(img(cv::Rect(0,0,300,300)), mask);
    //     cv::imshow("Test", img);
    //     cv::waitKey(1);
    // }

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
    // std::cout << "Time difference = " << 1200. / (static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count()) / 1e9) << "[fps]" << std::endl;

}