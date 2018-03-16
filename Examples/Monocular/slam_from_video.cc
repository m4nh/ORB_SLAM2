/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <Converter.h>
#include <System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

double currentTime() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return double(std::chrono::duration_cast<std::chrono::milliseconds>(now)
                    .count()) /
         1000.0;
}

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings movie_file"
         << endl;
    return 1;
  }

  using namespace cv;

  std::string filename = string(argv[3]);
  VideoCapture capture(filename);
  Mat frame;

  if (!capture.isOpened())
    throw "Error when reading steam_avi";

  // namedWindow( "w", 1);
  // for( ; ; )
  // {
  //     capture >> frame;
  //     if(frame.empty())
  //         break;
  //     std::cout << std::setprecision (15) <<currentTime() <<std::endl;
  //     imshow("w", frame);
  //     waitKey(20); // waits to display frame
  // }
  // waitKey(0); // key press to close window

  // exit(0);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;

  std::map<double, cv::Mat> images_map;
  // Main loop
  cv::Mat im;
  for (;;) {
    capture >> im;
    if (im.empty()) {
      printf("END TRACKING!\n");
      break;
    }
    { printf("TRACKING!\n"); }

    // Read image from file
    double tframe = currentTime();

    // std::chrono::steady_clock::time_point t1 =
    // std::chrono::steady_clock::now();

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);
    images_map[tframe] = im.clone();
    // std::chrono::steady_clock::time_point t2 =
    // std::chrono::steady_clock::now();

    // double ttrack= std::chrono::duration_cast<std::chrono::duration<double>
    // >(t2 - t1).count();

    // vTimesTrack[ni]=ttrack;

    // // Wait to load the next frame
    // double T=0;
    // if(ni<nImages-1)
    //     T = vTimestamps[ni+1]-tframe;
    // else if(ni>0)
    //     T = tframe-vTimestamps[ni-1];

    // if(ttrack<T)
    //     usleep((T-ttrack)*1e6);
  }

  printf("SHUTTING DOWN!\n");
  // Stop all threads
  SLAM.Shutdown();

  // // Tracking time statistics
  // sort(vTimesTrack.begin(),vTimesTrack.end());
  // float totaltime = 0;
  // for(int ni=0; ni<nImages; ni++)
  // {
  //     totaltime+=vTimesTrack[ni];
  // }
  // cout << "-------" << endl << endl;
  // cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
  // cout << "mean tracking time: " << totaltime/nImages << endl;

  printf("Saving!\n");
  // Save camera trajectory
  std::string output_filename = "/tmp/pino/robot_poses.txt";
  std::string image_path = "/tmp/pino/images/";
  ofstream f;
  f.open(output_filename.c_str());
  f << fixed;

  // Keyframes
  std::vector<ORB_SLAM2::KeyFrame *> keyframes;
  SLAM.GetAllKeyframes(keyframes);

  for (size_t i = 0; i < keyframes.size(); i++) {
    ORB_SLAM2::KeyFrame *kf = keyframes[i];

    if (kf->isBad())
      continue;

    cv::Mat image = images_map[kf->mTimeStamp];

    std::stringstream ss;
    ss << image_path << std::setfill('0') << std::setw(5) << i << ".png";
    std::cout << "ID:" << kf->mTimeStamp << " --> " << ss.str() << "\n";
    cv::imwrite(ss.str(), image);

    cv::Mat R = kf->GetRotation().t();
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
    cv::Mat t = kf->GetCameraCenter();
    f << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1)
      << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2]
      << " " << q[3] << endl;
  }

  SLAM.SaveKeyFrameTrajectoryTUM("/tmp/KeyFrameTrajectory.txt");

  printf("Close\n");
  return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {
  ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  string s0;
  getline(f, s0);
  getline(f, s0);
  getline(f, s0);

  while (!f.eof()) {
    string s;
    getline(f, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}
