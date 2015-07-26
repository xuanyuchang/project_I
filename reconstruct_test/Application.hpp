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

#ifndef __APPLICATION_HPP__
#define __APPLICATION_HPP__

#ifndef WINVER
#define WINVER 0x0500
#define _WIN32_WINNT 0x0500
#endif

#include <QApplication>

#include <QList>
#include <QFileSystemModel>
#include <QMap>

#include <opencv2/core/core.hpp>




#include "CalibrationData.hpp"
#include "scan3d.hpp"




enum Role {ImageFilenameRole = Qt::UserRole, GrayImageRole, ColorImageRole, 
           ProjectorWidthRole, ProjectorHeightRole};

#define WINDOW_TITLE "3D Scanning Software"
#define APP_NAME "scan3d-capture"


//decode
#define THRESHOLD_CONFIG    "decode/threshold"
#define THRESHOLD_DEFAULT   25
#define ROBUST_B_CONFIG     "decode/b"
#define ROBUST_B_DEFAULT    0.5
#define ROBUST_M_CONFIG     "decode/m"
#define ROBUST_M_DEFAULT    5

//checkerboard size
#define DEFAULT_CORNER_X        7
#define DEFAULT_CORNER_Y        11
#define DEFAULT_CORNER_WIDTH    21.08
#define DEFAULT_CORNER_HEIGHT   21.00

//calibration
#define HOMOGRAPHY_WINDOW_CONFIG         "calibration/homography_window"
#define HOMOGRAPHY_WINDOW_DEFAULT        60

//reconstruction
#define MAX_DIST_CONFIG         "reconstruction/max_dist"
#define MAX_DIST_DEFAULT        100.0
#define SAVE_NORMALS_CONFIG     "reconstruction/save_normals"
#define SAVE_NORMALS_DEFAULT    true
#define SAVE_COLORS_CONFIG      "reconstruction/save_colors"
#define SAVE_COLORS_DEFAULT     true
#define SAVE_BINARY_CONFIG      "reconstruction/save_binary"
#define SAVE_BINARY_DEFAULT     true

class Application : public QApplication
{
    Q_OBJECT
public:
    Application(int & argc, char ** argv);
    ~Application();

    void loadimage(void);
    cv::Mat get_image(unsigned level,unsigned i=0,Role role = GrayImageRole);
    bool extract_chessboard_corners(void);
    int get_projector_width(void);
    int get_projector_height(void);
    bool decode_gray_set(unsigned level, cv::Mat & pattern_image, cv::Mat & min_max_image);
    void calibrate(void);

    void load_calib(void);
    void reconstruct(void);

    void reconstruct_model(int level, scan3d::Pointcloud & pointcloud);
    void compute_normals(scan3d::Pointcloud & pointcloud);
    //data dir
//    void set_root_dir(const QString & dirname);
//    QString get_root_dir(void) const;
//    bool change_root_dir(QWidget * parent_widget = NULL);

//    void clear(void);

//    const cv::Mat get_image(unsigned level, unsigned n, Role role = GrayImageRole) const;
//    int get_camera_width(unsigned level = 0) const;
//    int get_camera_height(unsigned level = 0) const;
//    int get_projector_width(unsigned level = 0) const;
//    int get_projector_height(unsigned level = 0) const;

//    bool extract_chessboard_corners(void);
//    void decode_all(void);
//    void decode(int level, QWidget * parent_widget = NULL);
//    void calibrate(void);

//    bool decode_gray_set(unsigned level, cv::Mat & pattern_image, cv::Mat & min_max_image, QWidget * parent_widget = NULL) const;

//    void load_config(void);

//    //Detection/Decoding/Calibration processing
//    inline void processing_set_current_message(const QString & text) const {processingDialog.set_current_message(text); processEvents();}
//    inline void processing_reset(void) {processingDialog.reset(); processEvents();}
//    inline void processing_set_progress_total(unsigned value) {processingDialog.set_progress_total(value); processEvents();}
//    inline void processing_set_progress_value(unsigned value) {processingDialog.set_progress_value(value); processEvents();}
//    inline void processing_message(const QString & text) const {processingDialog.message(text); processEvents();}
//    inline bool processing_canceled(void) const {return processingDialog.canceled();}

//    //calibration
//    bool load_calibration(QWidget * parent_widget = NULL);
//    bool save_calibration(QWidget * parent_widget = NULL);

//    //reconstruction
//    void reconstruct_model(int level, scan3d::Pointcloud & pointcloud, QWidget * parent_widget = NULL);
//    void compute_normals(scan3d::Pointcloud & pointcloud);

//    void make_pattern_images(int level, cv::Mat & col_image, cv::Mat & row_image);
//    cv::Mat get_projector_view(int level, bool force_update = false);




public:

    std::vector<std::vector<cv::Mat> > _sum_pattern_to_corner;//图像数据源:(标定用的图像)
    //std::vector<cv::Mat> _patterns_to_corner;//图像数据源（子集）
    std::vector<cv::Mat> _image_to_reconstruct;//重建的图像集合

    CalibrationData calib;

    cv::Size chessboard_size;//棋盘上角点的个数
    cv::Size2f corner_size;//角点的大小
    std::vector<std::vector<cv::Point2f> > chessboard_corners;//检测到的角点位置坐标，原始图像上的
    std::vector<std::vector<cv::Point2f> > projector_corners;//投影仪中角点的编码值（角点在投影仪上的位置）
    std::vector<cv::Mat> pattern_list;//解码后的图像，已经将像素点分类好了；每一个Mat是两通道的矩阵，分别是垂直和水平的（码值的解法？？）编码值
    std::vector<cv::Mat> min_max_list;//每一对图案的灰度值的最大和最小的值
    std::vector<cv::Mat> projector_view_list;
    scan3d::Pointcloud pointcloud;
};

#define APP dynamic_cast<Application *>(Application::instance())

#endif  /* __APPLICATION_HPP__ */
