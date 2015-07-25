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




#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdlib>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "structured_light.hpp"


Application::Application(int & argc, char ** argv) : 
    QApplication(argc, argv),
    _sum_pattern_to_corner(),
    _patterns_to_corner(),
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
    for(int j=1;j<4;j++){///测试用2->4
        //QString dir="C://Users//Administrator//Desktop//calibrate_pic//";
        //QString num="%1//"
        for(int i=1;i<43;i++){///测试用1->0

            cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//calibrate_pic//%1//cam_%2.png").arg(j).arg(i, 2, 10, QLatin1Char('0')).toStdString());
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//original software make the patterns_10bit//pat_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//768*1024,源程序产生的pattern
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//for_calibration_1024//768&10bit//test//pat_%1.jpg").arg(i, 2, 10, QLatin1Char('0')).toStdString());//自己做得pattern:768
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//calibrate_pic//4//cam_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//标定例程图案：2；
            ///测试用cv::Mat image=cv::imread(QString("C://Users//jo//Desktop//decode//original software make the patterns_10bit//test//pat_%1.png").arg(i, 2, 10, QLatin1Char('0')).toStdString());//投影物体用的8bit的图案

            _patterns_to_corner.push_back(image);
        }
        _sum_pattern_to_corner.push_back(_patterns_to_corner);
        _patterns_to_corner.clear();
    }
}

cv::Mat Application::get_image(unsigned level,unsigned i)
{
    cv::Mat gray_image;
    cv::Mat rgb_image=_sum_pattern_to_corner.at(level).at(i);//////////////////////////////这个例子特殊，选取第二张找角点！！！！！！！！！
    //imshow(QString("%1").arg(level).toStdString(),rgb_image);
    cv::cvtColor(rgb_image, gray_image, CV_BGR2GRAY);
    return gray_image;

}

int Application::get_projector_width(void)//此函数可根据不同的投影仪尺寸进行修改……………………………………………………………………………………
{
    return 720;/*1024;*/
}

int Application::get_projector_height(void)//此函数可根据不同的投影仪尺寸进行修改……………………………………………………………………………………
{
    return 640;/*768;*/
}



bool Application::extract_chessboard_corners(void)
{
    //chessboard_size = cv::Size(7,11);//？？x,y方向是否搞…………………………………………………………
    //corner_size = cv::Size2f(21.08,21);//？？x,y方向是否搞反……………………………………………………

    unsigned count = static_cast<unsigned>(_sum_pattern_to_corner.size());

    chessboard_corners.clear();
    chessboard_corners.resize(count);//设置vector容器大小为子目录个数

    cv::Size imageSize(0,0);
    //int image_scale = 1;//??是否必要：要缩小图像

    bool all_found = true;
    for (unsigned i=0; i<count; i++)//只取每个子目录下的第一张图像
    {

        cv::Mat gray_image = get_image(i);//第一张图片转换为灰度图像，用opencv中的函数

        if (gray_image.rows<1)  //如果图像不存在，则跳过本子目录的处理
        {
            continue;//？？？？
        }
        if (imageSize.width==0)//初始化image size
        {   //init image size
            imageSize = gray_image.size();

        }
        else if (imageSize != gray_image.size())
        {   //error
            std::cout << "ERROR: image of different size: set " << i << std::endl;//如果图像集中的每一集的第一张图像的尺寸不同，则角点提取失败
            return false;
        }
        //this will be filled by the detected corners
        std::vector<cv::Point2f> & corners = chessboard_corners[i];//取出将要得到的角点的存入地址

        //oprncv的找角点函数
        if (cv::findChessboardCorners(gray_image, chessboard_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE /*+ cv::CALIB_CB_FILTER_QUADS*/))
        {

            std::cout << " - corners: " << corners.size() << std::endl;
        }
        else
        {
            all_found = false;

            std::cout << " - chessboard not found!" << std::endl;
        }

        if (corners.size())
        {
            //取角点的亚像素位置opencv书上354页
             cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                                cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }
        //画出角点在图像上
        //cv::drawChessboardCorners(gray_image, chessboard_size, corners, true);
        //imshow(QString("%1").arg(i).toStdString(),gray_image);
//        输出焦点坐标if(i==1)
//            std::cout<<corners<<std::endl;
   }
    return all_found;
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
    for(int i=0;i<42;i++)///42->38
    {
        images.push_back(get_image(level,i));

    }
    bool rv = sl::decode_pattern(images, pattern_image, min_max_image, projector_size, sl::RobustDecode|sl::GrayPatternDecode, direct_light, m);

    return rv;

}


void Application::calibrate(void)
{

     unsigned count = static_cast<unsigned>(_sum_pattern_to_corner.size());

     const unsigned threshold = 25;//需要修改…………………………………………………………
     calib.clear();//清除已经标定的结果

     cv::Size imageSize(0,0);//图片的尺寸
     //detect corners ////////////////////////////////////

     //找出角点

     if (!extract_chessboard_corners())
     {
         return;
     }


     //collect projector correspondences
     projector_corners.resize(count);
     pattern_list.resize(count);
     min_max_list.resize(count);

     for (unsigned i=0; i<count; i++)//遍历各个子文件夹

     {


         std::vector<cv::Point2f> const& corners = chessboard_corners[i];//取角点，存入chessboard_corners中
         std::vector<cv::Point2f> & pcorners = projector_corners[i];//定义对应的投影仪角点的引用

         pcorners.clear(); //erase previous points

         cv::Mat & pattern_image = pattern_list[i];
         cv::Mat & min_max_image = min_max_list[i];
         if (!decode_gray_set(i, pattern_image, min_max_image))//解码,并输出解码图案：pattern_image
         {   //error
             std::cout << "ERROR: Decode image set " << i << " failed. " << std::endl;
             return;
         }
         //验证各个pattern_list[i]的图片数量都相等
         if (imageSize.width==0)
         {
             imageSize = pattern_image.size();
         }
         else if (imageSize != pattern_image.size())
         {
             std::cout << "ERROR: pattern image of different size: set " << i << std::endl;
             return;
         }

         std::cout<<QString("Computing homographies... subset:NO.%1").arg(i).toStdString()<<std::endl;
         //计算每个摆放位置的（子文件的）角点对应的投影仪角点的homographies,一个点一个点的去找
         for (std::vector<cv::Point2f>::const_iterator iter=corners.begin(); iter!=corners.end(); iter++)
         {
             const cv::Point2f & p = *iter;//照相机上的角点
             cv::Point2f q;//投影仪上的角点

             //find an homography around p
             //找到homography
             unsigned WINDOW_SIZE = 30;
             std::vector<cv::Point2f> img_points, proj_points;

             if (p.x>WINDOW_SIZE && p.y>WINDOW_SIZE && p.x+WINDOW_SIZE<pattern_image.cols && p.y+WINDOW_SIZE<pattern_image.rows)//包含窗口的角点要不超过图像边界，否则标定失败
             {
                 for (unsigned h=p.y-WINDOW_SIZE; h<p.y+WINDOW_SIZE; h++)
                 {
                     register const cv::Vec2f * row = pattern_image.ptr<cv::Vec2f>(h);
                     register const cv::Vec2b * min_max_row = min_max_image.ptr<cv::Vec2b>(h);
                     //cv::Vec2f * out_row = out_pattern_image.ptr<cv::Vec2f>(h);
                     for (unsigned w=p.x-WINDOW_SIZE; w<p.x+WINDOW_SIZE; w++)
                     {
                         const cv::Vec2f & pattern = row[w];
                         const cv::Vec2b & min_max = min_max_row[w];
                         //cv::Vec2f & out_pattern = out_row[w];
                         if (sl::INVALID(pattern))//如果是uncertain点，则跳过此点的计算
                         {
                             continue;
                         }
                         if ((min_max[1]-min_max[0])<static_cast<int>(threshold))//如果一对图案的灰度值差小于阈值，则跳过此点的计算
                         {   //apply threshold and skip
                             continue;
                         }

                         img_points.push_back(cv::Point2f(w, h));//将选中的点集坐标压入栈
                         proj_points.push_back(cv::Point2f(pattern));//将选中的点集对应的垂直，水平的解码图案的码值压入栈

                         //out_pattern = pattern;
                     }
                 }
                 cv::Mat H = cv::findHomography(img_points, proj_points, CV_RANSAC);
                 //std::cout << " H:\n" << H << std::endl;
                 cv::Point3d Q = cv::Point3d(cv::Mat(H*cv::Mat(cv::Point3d(p.x, p.y, 1.0))));
                 //std::cout << " Q:\n" << Q << std::endl;
                 q = cv::Point2f(Q.x/Q.z, Q.y/Q.z);//Q.z=1;q为对应的编码值（垂直,水平）除以Q.z=>等比例放大，还原
             }
             else
             {
                 return;
             }
             //save
             pcorners.push_back(q);//编码值点压入栈

         }
             std::cout<<QString("  subset:NO. %1: get projector corners finished").arg(i).toStdString()<<std::endl;

     }

         //至此已经找出投影仪中（所有摆放位置的）对应的角点的垂直水平编码值
         //generate world object coordinates
         std::vector<cv::Point3f> world_corners;

         //std::cout<<chessboard_size.width<<corner_size.width<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
         //将棋盘上各个角点实际大小存入world_corners中，其中z轴坐标值为0，x,y轴存入的是各个角点的位置值（实际大小）
         //std::cout<<"chessboard_size.height:"<<chessboard_size.height<<"chessboard_size.width:"<<chessboard_size.width<<"corner_size.width:"<<corner_size.width<<"corner_size.height:"<<corner_size.height<<std::endl;
         //std::cout<<"chessboard_size:"<<chessboard_size<<"corner_size:"<<corner_size<<std::endl;
         for (int h=0; h<chessboard_size.height; h++)
         {
             for (int w=0; w<chessboard_size.width; w++)
             {
                 world_corners.push_back(cv::Point3f(corner_size.width * w, corner_size.height * h, 0.f));
             }
         }
         std::vector<std::vector<cv::Point3f> > objectPoints;//（全部子文件中的）角点的实际大小的坐标值集合
         std::vector<std::vector<cv::Point2f> > chessboard_corners_active;//（全部子文件中的）照相机获取的角点的坐标值
         std::vector<std::vector<cv::Point2f> > projector_corners_active;//（全部子文件中的）投影仪上角点的坐标值（垂直，水平编码值）

         objectPoints.reserve(count);//子文件夹个数count，本例中容器为3
         chessboard_corners_active.reserve(count);
         projector_corners_active.reserve(count);
         for (unsigned i=0; i<count; i++)//将各个角点坐标压入栈(world_corners，corners，pcorners),用于标定函数的输入参数
         {
             std::vector<cv::Point2f> const& corners = chessboard_corners.at(i);
             std::vector<cv::Point2f> const& pcorners = projector_corners.at(i);
             if (corners.size() == pcorners.size())
             {   //active set
                 objectPoints.push_back(world_corners);
                 chessboard_corners_active.push_back(corners);
                 projector_corners_active.push_back(pcorners);
             }
         }
           //std::cout<<objectPoints.at(0)<<std::endl<<chessboard_corners.at(0);
         if (objectPoints.size()<3)//棋盘位置小于3
         {
             std::cout<<"ERROR: use at least 3 sets"<<std::endl;

             return;
         }
         //opencv中用到的标定的标识？？？（什么意思）
         int cal_flags = 0
                       //+ cv::CALIB_FIX_K1
                       //+ cv::CALIB_FIX_K2
                       //+ cv::CALIB_ZERO_TANGENT_DIST
                       + cv::CALIB_FIX_K3
                       ;

         //calibrate the camera ////////////////////////////////////
         std::cout<<" * Calibrate camera"<<std::endl;
         std::vector<cv::Mat> cam_rvecs, cam_tvecs;
         int cam_flags = cal_flags;
         calib.cam_error = cv::calibrateCamera(objectPoints, chessboard_corners_active, imageSize, calib.cam_K, calib.cam_kc, cam_rvecs, cam_tvecs, cam_flags,
                                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

         //calibrate the projector ////////////////////////////////////
         std::cout<<" * Calibrate projector"<<std::endl;
         std::vector<cv::Mat> proj_rvecs, proj_tvecs;
         int proj_flags = cal_flags;
         cv::Size projector_size(get_projector_width(), get_projector_height());
         calib.proj_error = cv::calibrateCamera(objectPoints, projector_corners_active, projector_size, calib.proj_K, calib.proj_kc, proj_rvecs, proj_tvecs, proj_flags,
                                                  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

         //stereo calibration
         std::cout<<" * Calibrate stereo"<<std::endl;
         cv::Mat E, F;//(用处？？？矩阵代表的含义：看opencv函数的各个参数介绍)
         calib.stereo_error = cv::stereoCalibrate(objectPoints, chessboard_corners_active, projector_corners_active, calib.cam_K, calib.cam_kc, calib.proj_K, calib.proj_kc,
                                                     imageSize /*ignored*/, calib.R, calib.T, E, F,
                                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON),
                                                     cv::CALIB_FIX_INTRINSIC /*cv::CALIB_USE_INTRINSIC_GUESS + cal_flags*/);


         //print to console（在控制台上输出）
         calib.display();

         //save to file(保存文件opencv常用的文件格式)
         QString path = QString("C:/Users/jo/Desktop/calibrate_pic");/////文件保存的根目录
         QString filename = path + "/calibration.yml";
         if (calib.save_calibration(filename))
         {
             std::cout<<QString("Calibration saved: %1").arg(filename).toStdString()<<std::endl;
         }
         else
         {
             std::cout<<QString("[ERROR] Saving %1 failed").arg(filename).toStdString()<<std::endl;
         }

         //save to MATLAB format(保存文件Matlab常用的文件格式)
         filename = path + "/calibration.m";
         if (calib.save_calibration(filename))
         {
             std::cout<<QString("Calibration saved [MATLAB]: %1").arg(filename).toStdString()<<std::endl;
         }
         else
         {
             std::cout<<QString("[ERROR] Saving %1 failed").arg(filename).toStdString()<<std::endl;
         }

         //save corners（model.txt是物理角点，cam_00.txt是照相机角点，proj_01是投影仪角点）
         FILE * fp = NULL;
         filename = path + "/model.txt";
         fp = fopen(qPrintable(filename), "w");
         if (!fp)
         {
             std::cout << "ERROR: could no open " << filename.toStdString() << std::endl;
             return;
         }
         std::cout << "Saved " << filename.toStdString() << std::endl;
         for (std::vector<cv::Point3f>::const_iterator iter=world_corners.begin(); iter!=world_corners.end(); iter++)
         {
             fprintf(fp, "%lf %lf %lf\n", iter->x, iter->y, iter->z);
         }
         fclose(fp);
         fp = NULL;

         for (unsigned i=0; i<count; i++)
         {
             std::vector<cv::Point2f> const& corners = chessboard_corners.at(i);
             std::vector<cv::Point2f> const& pcorners = projector_corners.at(i);

             QString filename1 = QString("%1/cam_%2.txt").arg(path).arg(i, 2, 10, QLatin1Char('0'));
             FILE * fp1 = fopen(qPrintable(filename1), "w");
             if (!fp1)
             {
                 std::cout << "ERROR: could no open " << filename1.toStdString() << std::endl;
                 return;
             }
             QString filename2 = QString("%1/proj_%2.txt").arg(path).arg(i, 2, 10, QLatin1Char('0'));
             FILE * fp2 = fopen(qPrintable(filename2), "w");
             if (!fp2)
             {
                 fclose(fp1);
                 std::cout << "ERROR: could no open " << filename2.toStdString() << std::endl;
                 return;
             }

             std::cout << "Saved " << filename1.toStdString() << std::endl;
             std::cout << "Saved " << filename2.toStdString() << std::endl;

             std::vector<cv::Point2f>::const_iterator iter1 = corners.begin();
             std::vector<cv::Point2f>::const_iterator iter2 = pcorners.begin();
             for (unsigned j=0; j<corners.size(); j++, iter1++, iter2++)
             {
                 fprintf(fp1, "%lf %lf\n", iter1->x, iter1->y);
                 fprintf(fp2, "%lf %lf\n", iter2->x, iter2->y);
             }
             fclose(fp1);
             fclose(fp2);
         }



}
