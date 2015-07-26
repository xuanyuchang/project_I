/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Application.hpp"

#include <QFileDialog>


#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdlib>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "io_util.hpp"
#include "structured_light.hpp"


Application::Application(int & argc, char ** argv) : 
    QApplication(argc, argv),
    _sum_pattern_to_corner(),
    //_patterns_to_corner(),
    _image_to_reconstruct(),
    calib(),//标定变量的初始化，初始化图像矩阵数据： cv::Mat cam_K;
                                            // cv::Mat cam_kc;
                                            // cv::Mat proj_K;
                                            // cv::Mat proj_kc;
                                            //cv::Mat R;
                                            // cv::Mat T;
    chessboard_size(11, 7),        //初始化棋盘的大小，横纵分别为多少个角点
    corner_size(21.f, 21.08f),        //初始化角点矩形的长度
    chessboard_corners(),           //检测出的棋盘角点的位置的向量
    projector_corners(),            //检测出的（计算出的）投影仪图像中角点的位置的向量
    pattern_list(),                 //一系列的投影图案（原始照片）---或者是pattern images选项中的图像
    min_max_list(),//？？？不知道是哪个图片的list
    projector_view_list()          //将拍照下的原图像反映射到投影仪中的原始图像

{

}

Application::~Application()
{
}

void Application::loadimage(void){

        for(int i=0;i<38;i++){///测试用1->0

            //cv::Mat image=cv::imread(QString("C://Users//Administrator//Desktop//decode//calibration_test//patterns//%1//cam_%2.png").arg(j).arg(i, 2, 10, QLatin1Char('0')).toStdString());
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//original software make the patterns_10bit//pat_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//768*1024,源程序产生的pattern
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//for_calibration_1024//768&10bit//test//pat_%1.jpg").arg(i, 2, 10, QLatin1Char('0')).toStdString());//自己做得pattern:768
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//calibrate_pic//4//cam_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//标定例程图案：2；
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//original software make the patterns_10bit//test//pat_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//投影物体用的8bit的图案
            // cv::Mat image=cv::imread(QString("D://Qt_test//build-Project02_3D-Desktop_Qt_5_2_1_MinGW_32bit-Debug//9bit2//pat_%2.jpg").arg(i, 2, 10, QLatin1Char('0')).toStdString());
            cv::Mat image=cv::imread(QString("C://Users//Administrator//Desktop//111//pattern//capture_result//test2//reconstruct//cam_%2.jpg").arg(i, 2, 10, QLatin1Char('0')).toStdString());

           _image_to_reconstruct.push_back(image);
        }
        _sum_pattern_to_corner.push_back(_image_to_reconstruct);
        _image_to_reconstruct.clear();
        std::cout<<"load image success!"<<std::endl;

}

cv::Mat Application::get_image(unsigned level, unsigned i, Role role)
{
    if (role!=GrayImageRole && role!=ColorImageRole)
    {   //invalid args
        return cv::Mat();
    }
    if (role==ColorImageRole)
    {
        return _sum_pattern_to_corner.at(level).at(i);
    }
    else
    {
        cv::Mat gray_image;
        cv::Mat rgb_image=_sum_pattern_to_corner.at(level).at(i);//////////////////////////////这个例子特殊，选取第二张找角点！！！！！！！！！
        //imshow(QString("%1").arg(level).toStdString(),rgb_image);
        cv::cvtColor(rgb_image, gray_image, CV_BGR2GRAY);
        return gray_image;
    }


}

int Application::get_projector_width(void)//此函数可根据不同的投影仪尺寸进行修改……………………………………………………………………………………
{
    return 1024;
}

int Application::get_projector_height(void)//此函数可根据不同的投影仪尺寸进行修改……………………………………………………………………………………
{
    return 768;
}





bool Application::decode_gray_set(unsigned level, cv::Mat & pattern_image, cv::Mat & min_max_image)
{
    pattern_image = cv::Mat();
    min_max_image = cv::Mat();
    //processEvents();
    //parameters
    const float b = 0.3;
    const unsigned m = 5;
    //estimate direct component
    std::vector<cv::Mat> images;
    int total_images = _sum_pattern_to_corner.at(0).size();//图像总数：42
    int total_patterns = total_images/2 - 1;//投影图案数目为20个

    const int direct_light_count = 4;
    const int direct_light_offset = 4;
    if (total_patterns<direct_light_count+direct_light_offset)//估算direct component 所需的图片数是八个，要求20（本算法要求使用的个数）>8(所需的投影pattern的个数);
    {   //too few images

        std::cout << "ERROR: too few pattern images" << std::endl;
        return false;
    }

    QList<unsigned> direct_component_images;//存倒数第8张到倒数第4张图像及其对应的反编码的图像，共8张，用这些图像图estimate direct component
    for (unsigned i=0; i<direct_light_count; i++)
    {
        int index = total_images - total_patterns - direct_light_count - direct_light_offset + i + 1;
        direct_component_images.append(index);
        direct_component_images.append(index + total_patterns);
    }
    //QList<unsigned> direct_component_images(QList<unsigned>() << 15 << 16 << 17 << 18 << 35 << 36 << 37 << 38);
                                                               //？？？？<<15 << 35 << 16 << 36......
    foreach (unsigned i, direct_component_images)
    {
        images.push_back(get_image(level, i-1));//将选取的灰度图像存入容器images中
    }
    //estimate direct component函数//输出：cv::Mat direct_light是二通道的图像，第一通道是Ld,第二通道是Lg(iTotal？???)
    cv::Mat direct_light = sl::estimate_direct_light(images, b);

    std::cout<<"Estimate direct and global light components... done."<<std::endl;
    std::cout << "[decode_set " << level << std::endl;
    std::cout<<"Decoding, please wait..."<< std::endl;
    cv::Size projector_size(get_projector_width(), get_projector_height());//投影仪的尺寸

    //解码图案,且pattern_image此时已是解码后的图像，常规二进制编码值，对于uncertain像素点，赋值为【1.#QNAN】

    //输入image_names是42张图像的灰度图像；??但，函数中，是否使用了前两张图片？？？？？
    images.clear();//在这里将其中转换为灰度图像；
    for(int i=0;i<38;i++)///42->38
    {
        images.push_back(get_image(level,i));

    }
    bool rv = sl::decode_pattern(images, pattern_image, min_max_image, projector_size, sl::RobustDecode|sl::GrayPatternDecode, direct_light, m);

    return rv;

}


void Application::load_calib(void)//加载标定结果
{
    calib.load_calibration(QString("C://Users//Administrator//Desktop//decode//calibration_test//patterns//calibration.yml"));
    std::cout<<"load_calib......"<<std::endl;
}

void Application::compute_normals(scan3d::Pointcloud & pointcloud)
{
    scan3d::compute_normals(pointcloud);
}



void Application::reconstruct(void)//对应mainwindow.cpp中的函数
{

    bool normals = false;
    bool colors = true;
    bool binary = true;


    scan3d::Pointcloud & pointcloud = APP->pointcloud;
    reconstruct_model(0, pointcloud);//这里只用了一个集合->去进行重建;i=0
    if (!pointcloud.points.data)
    {   //no points: reconstruction canceled or failed
        std::cout<<"Reconstruction failed"<<std::endl;
        return;
    }
    if (normals)
    {
        //busy cursor
        std::cout<<"Computing normals..."<<std::endl;
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        QApplication::processEvents();

        APP->compute_normals(pointcloud);

        //restore regular cursor
        QApplication::restoreOverrideCursor();
        QApplication::processEvents();
    }


    QString filename = "C://Users//Administrator//Desktop//111//pattern//capture_result//test2//reconstruct//2015-7-26.ply";
   // QString name="C://Users//Administrator//Desktop//111//pattern//capture_result//test2//reconstruct//2015-7-26";
    //QString filename = QFileDialog::getSaveFileName(this, "Save pointcloud", name+".ply", "Pointclouds (*.ply)");
    if (!filename.isEmpty())
    {
        //busy cursor
        std::cout<<QString("Saving to %1...").arg(filename).toStdString()<<std::endl;
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        QApplication::processEvents();

        unsigned ply_flags = io_util::PlyPoints
                            | (colors?io_util::PlyColors:0)
                            | (normals?io_util::PlyNormals:0)
                            | (binary?io_util::PlyBinary:0);

        io_util::write_ply(filename.toStdString(), pointcloud, ply_flags);

        //restore regular cursor
        QApplication::restoreOverrideCursor();
        QApplication::processEvents();

        std::cout << QString("Pointcloud saved: %1").arg(filename).toStdString() << std::endl;
    }

}


void Application::reconstruct_model(int level, scan3d::Pointcloud & pointcloud)//对应Application中的函数
{
   // std::cout<<"in the reconstruct function......"<<std::endl;

    if(_sum_pattern_to_corner.at(level).size()!=38)//要改：这里暂时用38张图片来重建
    {
        std::cout<<"image number is not right!"<<std::endl;
        return;
    }
    if (!calib.is_valid())
    {   //invalid calibration
        std::cout<<"Error:"<< "No valid calibration found."<<std::endl;
        return;
    }

    //decode first
    //这里只用了一个集合->去进行重建;i=0
    int i=level;

    pattern_list.resize(1);//要改
    min_max_list.resize(1);//要改

    cv::Mat & pattern_image = pattern_list[i];
    cv::Mat & min_max_image = min_max_list[i];


    if (!decode_gray_set(i, pattern_image, min_max_image))//解码,并输出解码图案：pattern_image
    {   //error
        std::cout << "ERROR: Decode image set " << i << " failed. " << std::endl;
        return;
    }
    cv::Mat pattern_image_1 = pattern_list.at(level);
    cv::Mat min_max_image_1 = min_max_list.at(level);;
    cv::Mat color_image_1 = get_image(level, 0, ColorImageRole);
    //cv::imshow("hello",color_image_1);
    if (!pattern_image_1.data || !min_max_image_1.data)
    {   //error: decode failed
        return;
    }
    cv::Size projector_size(get_projector_width(), get_projector_height());
    int threshold = 25;
    double max_dist = 100;

    //进一步调用重构函数
    scan3d::reconstruct_model(pointcloud, calib, pattern_image_1, min_max_image_1, color_image_1, projector_size, threshold, max_dist);

    //cv::imshow("hello",pointcloud.points);
    //cv::imshow("hello1",pointcloud.colors);
    //保存物体的projector_view图像
    //pointcloud.colors.copyTo(projector_view_list[level]);
}



















