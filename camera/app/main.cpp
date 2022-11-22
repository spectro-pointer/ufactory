// Executables must have the following defined if the library contains
// doctest definitions. For builds with this disabled, e.g. code shipped to
// users, this can be left out.
#ifdef ENABLE_DOCTEST_IN_LIBRARY
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#endif

#include <librealsense2/rs.hpp>

#include <vector>
#include <iostream>
#include <numeric>
#include <execution>
#include <stdlib.h>
#include <algorithm>            // std::min, std::max

#include "exampleConfig.h"
#include "main.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wimplicit-int-conversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma GCC diagnostic ignored "-Wsuggest-destructor-override"
#pragma GCC diagnostic ignored "-Wsuggest-override"
#pragma GCC diagnostic ignored "-Wdeprecated-dynamic-exception-spec"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wextra-semi"
#pragma GCC diagnostic ignored "-Wimplicit-float-conversion"
#pragma GCC diagnostic ignored "-Wweak-vtables"
#pragma GCC diagnostic ignored "-Wdeprecated-dynamic-exception-spec"
#include <xarm/wrapper/xarm_api.h>
#pragma GCC diagnostic pop

/*
static int reset(XArmAPI *arm) {
  float pos[6] = {0.0f, -56.8f, -35.0f, 91.8f, 0.0f};
  int code = arm->set_servo_angle(pos, true, 5.0f, 1.0f);
  return code;
  
    (code, point) = arm.get_initial_point()   # toma el aggulo del programa 
    if code != 0:
        print('get_initial_point error: {}'.format(code))
        exit(1)

    code = arm.set_servo_angle(angle=point, is_radian=False, wait=True)
    if code != 0:
        print('set_servo_angle error: {} {}'.format(code, point))
        arm.disconnect()
        exit(1)
}*/

float average(std::vector<float> const& v);
float average(std::vector<float> const& v){
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<float>(v.size());
    return std::reduce(v.begin(), v.end()) / count;
}

int main() {
  std::cout << "C++ xArm RealSense Apple Picker v"
            << PROJECT_VERSION_MAJOR
            << "."
            << PROJECT_VERSION_MINOR
            << "."
            << PROJECT_VERSION_PATCH
            << "-"
            << PROJECT_VERSION_TWEAK
            << std::endl;

	std::string robot_port = "192.168.1.225";

	XArmAPI *arm = new XArmAPI(robot_port);
	sleep_milliseconds(500);

  arm->motion_enable(true);
  arm->set_mode(0);
  arm->set_state(0);

  /*int code = reset(arm);
  if(code != 0){
    // couldn't reset
    std::cout << "An error occured during the reset of the robot arm.\n"
              << "Error Code: "
              << code
              << "\n";
  }*/

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  rs2::config config;
  rs2::colorizer color_map;
  config.enable_stream(rs2_stream::RS2_STREAM_COLOR, 640, 480, rs2_format::RS2_FORMAT_BGR8, 30);
  config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 640, 480, rs2_format::RS2_FORMAT_Z16, 30);
  // Start streaming with default recommended configuration
  pipe.start(config);

  // Capture 30 frames to give autoexposure, etc. a chance to settle
  // for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
  
  // don't open the window before being sure that the camera works
  pipe.wait_for_frames();

  const auto window_name = "Apple Picker";
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Apple Box Mask", cv::WINDOW_AUTOSIZE);

  auto aruco_dict = cv::aruco::Dictionary::get(cv::aruco::DICT_5X5_50);
  auto aruco_parameters = cv::aruco::DetectorParameters::create();

  while (static_cast<char>(cv::waitKey(1)) != 27){
    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto color_frame = frames.get_color_frame();
    auto depth = frames.get_depth_frame();
    auto depth_frame = depth.apply_filter(color_map);
    
    const int w = color_frame.get_width();
    const int h = color_frame.get_height();

    cv::Mat color_image(
      cv::Size(w, h),
      CV_8UC3,
      const_cast<void*>(color_frame.get_data()),
      cv::Mat::AUTO_STEP
    );

    cv::Mat depth_image(
      cv::Size(w, h),
      CV_8UC3,
      const_cast<void*>(depth_frame.get_data()),
      cv::Mat::AUTO_STEP
    );

    std::vector<int> aruco_ids;
    std::vector<std::vector<cv::Point2f>> aruco_corners;

    cv::aruco::detectMarkers(
      color_image,
      aruco_dict,
      aruco_corners,
      aruco_ids,
      aruco_parameters
    );

    if(aruco_ids.size() > 0)cv::aruco::drawDetectedMarkers(
      color_image,
      aruco_corners,
      aruco_ids
    );

    if(aruco_ids.size() == 4){
      auto process = true;
      int seen[4] = {0, 0, 0, 0};

      // make sure we only have one of each marker
      // and they're each under 4
      for(int i = 0; i < 4; i++){
        auto id = seen[i] = aruco_ids[static_cast<std::vector<int>::size_type>(i)];
        if(id > 3){
          process = false;
          break;
        }

        for(int j = 0; j < i; j++){
          if(seen[i] == seen[j]){
            process = false;
            break;
          }
        }

        if(!process) break;
      }

      if(process){
        cv::Point corners_points[4];

        for(int i = 0; i < 4; i++){
          auto id = aruco_ids[static_cast<std::vector<int>::size_type>(i)];
          auto corners = aruco_corners[static_cast<std::vector<std::vector<cv::Point2f>>::size_type>(i)];
          cv::Point topLeft = corners[0];
          cv::Point bottomRight = corners[2];

          cv::Point corner(
            topLeft.x + (topLeft.x - bottomRight.x) / 7,
            topLeft.y + (topLeft.y - bottomRight.y) / 7
          );

          cv::circle(
            color_image,
            corner,
            4,
            cv::Scalar(0, 0, 255),
            -1
          );
          
          corners_points[id] = corner;
        }

        std::vector<cv::Point> countour;
        for(int i = 0; i < 4; i++){
          cv::line(
            color_image,
            corners_points[i],
            corners_points[(i + 1) % 4],
            cv::Scalar(0, 255, 0),
            2
          );

          countour.push_back(corners_points[i]);
        }

        std::vector<std::vector<cv::Point>> allCountours;
        allCountours.push_back(countour);

        cv::Mat mask(
          cv::Size(w, h),
          CV_8UC1,
          cv::Scalar(0)
        );
        cv::fillPoly(
          mask,
          allCountours,
          cv::Scalar(255)
        );

        cv::imshow("Apple Box Mask", mask);


        std::vector<float> grid_distances;
        unsigned char *p;
        p = mask.data;
        auto pixels = mask.rows*mask.cols;
        // we now need to find the point where the robot can put the apple
        for(int pos = 0; pos < pixels; pos += 10){
          auto y = pos / mask.cols;
          auto x = pos % mask.cols;
          // for each pixel of the image
          //auto pixel = static_cast<int>(p[pos]);
          //if(pixel <= 0)continue;

          /*
          // if the pixel is in the mask
          // make a circular mask
          cv::Mat circle_mask(
            cv::Size(w, h),
            CV_8UC1,
            cv::Scalar(0)
          );
          cv::circle(
            circle_mask,
            cv::Point(x, y),
            radius,
            cv::Scalar(255),
            -1
          );
          cv::Mat result(
            cv::Size(w, h),
            CV_8UC1,
            cv::Scalar(0)
          );
          cv::bitwise_and(mask, mask, result, circle_mask);


          unsigned char *p2;
          p2 = result.data;
          int fromx = std::max(x - radius, 0);
          int fromy = std::max(y - radius, 0);
          int tox = std::min(x + radius, mask.cols);
          int toy = std::min(y + radius, mask.rows);*/
          std::vector<float> distances;

          for(int x2 = x; x2 < x+10; x2++){
            for(int y2 = y; y2 < y+10; y2++){
              auto pos2 = y2 * mask.cols + x2;
              auto pixel2 = static_cast<int>(p[pos2]);
              if(pixel2 <= 0)continue;
              
              auto dist = depth.get_distance(x2, y2);
              if(dist <= 0.30f)continue;
              distances.push_back(dist);
            }
          }

          float average_dist = average(distances);
          grid_distances.push_back(average_dist);
        }

        // now we need to find the lowest/furthest point
        auto mint = grid_distances.at(0);
        int i = 0;
        for(int j = 1; j < grid_distances.size(); j++){
          if(grid_distances.at(j) > mint){
            mint = grid_distances.at(j);
            i = j;
          }
        }

        if(mint == 0){
          std::cout << "No valid point found" << std::endl;
        }else{
          auto y = i / mask.cols + 5;
          auto x = i % mask.cols + 5;
          auto dist = depth.get_distance(x, y);
          std::cout << "Point: " << x << ", " << y << " Distance: " << dist << std::endl;
        }
      }
    }

    cv::Mat result;

    cv::hconcat(color_image, depth_image, result);

    cv::imshow(window_name, result);
  }

  return 0;
}
