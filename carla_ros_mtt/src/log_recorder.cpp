//
// Created by pedro on 14-04-2019.
//

/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/**
\file
\brief Convert a laser scan in ros format to a text ASCII file
*/

// System Includes
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.8/pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <fstream>

using namespace std;
using namespace ros;

string outputFile="~/Desktop/temp.txt";

void savePointCloud2(const sensor_msgs::PointCloud2& cloud)
{
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(cloud, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, pclCloud);

    ofstream file(outputFile.c_str(),ios::app);

    //get points into grid to reorder them
    for(uint i=0;i<pclCloud.points.size();i++)
        file<<pclCloud.points[i].x<<" "<<pclCloud.points[i].y<<" ";

    file<<cloud.header.stamp<<endl;

    return;
}

void saveLaserScan(const sensor_msgs::LaserScan& scan)
{
    ofstream file(outputFile.c_str(),ios::app);

    double min=0.1;//10 cm
    double max=50.;//50 m
    static bool init=false;

    if(init)
    {
        //write header
        file<<"# ros-bag to log converter"<<endl;
        file<<"# Jorge Almeida"<<endl;
        file<<"# Formato do ficheiro:"<<endl;
        file<<"# cada linha representa um scan diferente"<<endl;
        file<<"# é assumido que todos os pontos num scan foram obtidos ao mesmo tempo"<<endl;
        file<<"# o instante de tempo a que o scan corresponde aparece sempre na ultima coluna de uma linha"<<endl;
        file<<"# todos os scans têm o mesmo número de pontos"<<endl;
        file<<"# os valores aqui apresentados correspondem às distancias em metros"<<endl;
        file<<"# os valores de distancia são sempre positivos, um valor negativo é uma medida falhada"<<endl;
        file<<"# as distancias correspondem a ângulos de medição consecutivos começando em angle_min e terminando em angle_max"<<endl;
        file<<"# os valores desses ângulos vêm em radianos"<<endl;
        file<<"# podes calcular a frequência dos scans usando o tempo de cada um, o valor de scan_time é o valor médio/ótimo (tempo entre scans)"<<endl;
        file<<"# angle_min: "<<scan.angle_min<<endl;
        file<<"# angle_max: "<<scan.angle_max<<endl;
        file<<"# scan_time: "<<scan.scan_time<<endl;

        init=false;
    }

    for(uint i=0;i<scan.ranges.size();i++)
    {
        if(scan.ranges[i]<min || scan.ranges[i]>max)
            file<<-1<<" ";
        else
            file<<scan.ranges[i]<<" ";
    }
    file<<scan.header.stamp<<endl;
}

int main(int argc,char**argv)
{
    // Initialize ROS
    init(argc,argv,"mht");

    NodeHandle nh("~");

    Subscriber laser_handler = nh.subscribe("/scan", 1000, saveLaserScan);
    Subscriber point_handler = nh.subscribe("/points", 1000, savePointCloud2);

    nh.getParam("output", outputFile);

    if(remove(outputFile.c_str()) != 0 )
        perror( "Error deleting file" );

    cout<<"Writing to "<<outputFile<<endl;
    cout<<"Subscribed to "<<names::remap("/scan")<<endl;
    cout<<"Subscribed to "<<names::remap("/points")<<endl;

    cout<<"Spinning"<<endl;
    spin();
    return 0;
}