/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"
#include "def.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

string type2str(int type)
{
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

double trackTs = 0;
int main(int argc, char *argv[])
{

    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        debug_print("File name: %s", file_name.c_str());
    }

    // Load all sequences:
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages = 0;
    int nImu = 0;
    int first_imu = 0;

    string pathSeq(argv[3]);
    string pathTimeStamps(argv[4]);

    string pathCam0 = pathSeq + "/mav0/cam0/data";
    string pathImu = pathSeq + "/mav0/imu0/data.csv";

    debug_print("pathCam0: %s", pathCam0.c_str());
    debug_print("pathImu: %s", pathImu.c_str());
    debug_print("pathTimeStamps: %s", pathTimeStamps.c_str());

    cout << "Loading images for sequence ...";
    LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames, vTimestampsCam);
    cout << "LOADED!" << endl;

    cout << "Loading IMU for sequence ...";
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "LOADED!" << endl;

    nImages = vstrImageFilenames.size();
    nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
      cerr << "ERROR: Failed to load images or IMU for sequence" << endl;
      return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first
    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
      first_imu++;
    first_imu--; // first imu measurement to be considered
    debug_print("nImages %d, nImu %d", nImages, nImu);
    for(int i = 0; i < 10; i++) debug_print("ImuTs %f, CamTs %f", vTimestampsImu[i], vTimestampsCam[i]);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);

    int processedImg = 0;
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for(int curImg = 0; curImg < nImages; curImg++, processedImg++)
    {
      im = cv::imread(vstrImageFilenames[curImg], cv::IMREAD_UNCHANGED);

      string imgType = type2str(im.type());
      //debug_print("Matrix type %s, %dx%d", imgType.c_str(), im.cols, im.rows);

      double tframe = vTimestampsCam[curImg];

      if(im.empty())
      {
        cerr << endl << "Failed to load image at: " <<  vstrImageFilenames[curImg] << endl;
        return 1;
      }

      // Load imu measurements from previous frame
      vImuMeas.clear();

      if(curImg>0)
      {
        while(vTimestampsImu[first_imu]<=vTimestampsCam[curImg])
        {
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,
                                                   vAcc[first_imu].y,
                                                   vAcc[first_imu].z,
                                                   vGyro[first_imu].x,
                                                   vGyro[first_imu].y,
                                                   vGyro[first_imu].z,
                                                   vTimestampsImu[first_imu]));
          first_imu++;
        }
      }

      double st = getTsNow();
      debug_print("curImg %d, imus %d", curImg, first_imu);
      cv::Mat curPose = SLAM.TrackMonocular(im,tframe,vImuMeas);
      std::string t = type2str(curPose.type());
      if(curPose.empty() == false)
      {
        cout << "curPose " << t << " = " << endl << " "  << curPose << endl << endl;
      }

      double et = getTsNow();

#ifdef REGISTER_TIMES
      SLAM.InsertTrackTime(et-st);
#endif
      //debug_print("Exetime for each frame with %ld Imus: %f ms", vImuMeas.size(), et-st);

      double exeUs = (et-st)/1000;
      trackTs += exeUs;

      vTimesTrack[curImg] = trackTs;

      // Wait to load the next frame
      double waitTs = 0;
      if(curImg < nImages-1)
        waitTs = vTimestampsCam[curImg+1]-tframe;
      else if(curImg > 0)
        waitTs = tframe - vTimestampsCam[curImg-1];

      //debug_print("extUs: %f waitTs: %f", exeUs, waitTs);
      if(exeUs < waitTs)
        usleep((waitTs - exeUs)*1e6);
    }

    //SLAM.ChangeDataset();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
