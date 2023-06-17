#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QDebug>
#include<eigen3/Eigen/Dense>
#include <QCoreApplication>
#include "osqp_problem.h"
#include <math.h>
#include <iostream>
#include <array>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace  std;
using PathBoundary = std::vector<std::pair<double, double> >;
using namespace Eigen;


class PathBoundaryInfo {
 public:
  inline void set_start_s(double start_s) { start_s_ = start_s; }
  inline void set_delta_s(double delta_s) { delta_s_ = delta_s; }
  inline void set_boundary(const PathBoundary& boundary) { boundary_ = boundary; }

  inline double delta_s() { return delta_s_; }
  inline const PathBoundary& boundary() { return boundary_; }
  PathBoundary boundary_;

 private:
  double start_s_;
  double delta_s_;

};



float getGPSDistance(double lat1, double lng1, double lat2, double lng2)
{
    float disn;
    double _x,_y;
    _x=fabs(lng1-lng2)*111700*cos(lat1/180*M_PI);
    _y=fabs(lat1-lat2)*111700;

    disn=sqrt(_x*_x+_y*_y);
    return disn;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

}

void MainWindow::initMainPathFiles()
{
    GPSInfo gps_temp;
    QString load_file_path;
//    QString main_path_name = QFileDialog::getOpenFileName(this,tr("主路径文件"),
//                                                           load_file_path,tr("边界文件(*.lpx)"));//打开文件

    QString main_path_name = "/home/jiaotong404/speed_plan/speed_safe_qp/build/HUO_path/empty/F_1_0_0.lpx";
    if(main_path_name.isEmpty())
        return;

    load_file_path = main_path_name;
    QString main_path_data = TextStreamRead(main_path_name);
    QStringList row_datas = main_path_data.split("\n");
    main_path_gps.clear();
    dis_lon_main_path.clear();
    dis_lat_main_path.clear();
    pitch.clear();
    q_pitch.clear();

    for(int i = 0; i < row_datas.size();i++)
    {
        QString row_data = row_datas.at(i);
        QStringList row_data_sprit = row_data.split(",");

//        if(row_data_sprit.size() != 6)
//            continue;
//        gps_temp.iHead = row_data_sprit.at(4).toDouble();
//        gps_temp.iLatitude = row_data_sprit.at(1).toDouble();
//        gps_temp.iLongitude = row_data_sprit.at(2).toDouble();
//        gps_temp.altitude = row_data_sprit.at(3).toDouble();

        if(row_data_sprit.size() != 14)
            continue;
        gps_temp.iHead = row_data_sprit.at(2).toDouble();
        gps_temp.iLatitude = row_data_sprit.at(3).toDouble();
        gps_temp.iLongitude = row_data_sprit.at(4).toDouble();
        gps_temp.altitude = row_data_sprit.at(7).toDouble();

        main_path_gps.push_back(gps_temp);
        dis_lat_main_path.append(gps_temp.iLatitude);
        dis_lon_main_path.append(gps_temp.iLongitude);
        dis_alt_main_path.append(gps_temp.altitude);
        manual_design_spd.append(row_data_sprit.at(8).toDouble());
    }

   initLogPathFiles();     //load the driving log
}


//加载日志文件
void MainWindow::initLogPathFiles()
{
    QString load_file_path;
    QString main_path_name = "/home/jiaotong404/speed_plan/speed_safe_qp/build/manual.txt";

//    QString main_path_name = QFileDialog::getOpenFileName(this,tr("日志文件"), load_file_path,tr("边界文件(*.txt)"));//打开文件
//    if(main_path_name.isEmpty())
//        return;

    load_file_path = main_path_name;
    QString main_path_data = TextStreamRead(main_path_name);
    QStringList row_datas = main_path_data.split("\n");
    dis_lat_log_path.clear();
    dis_lon_log_path.clear();
    dis_spd_log_path.clear();

    for(int i = 0; i < row_datas.size();i++)
    {
        QString row_data = row_datas.at(i);
        QStringList row_data_sprit = row_data.split(",");

        if(row_data_sprit.size() != 3)
            continue;

        double temp_lat = row_data_sprit.at(0).toDouble();
        double temp_lon = row_data_sprit.at(1).toDouble();
        double temp_spd = row_data_sprit.at(2).toDouble();
        dis_lat_log_path.append(temp_lat);
        dis_lon_log_path.append(temp_lon);
        dis_spd_log_path.append(temp_spd/3.6/1.4);
    }
}

//东北天坐标系转换
void MainWindow::TransToENU()
{
    double psiz, psiy, x_I, y_I, z_I;
    double M_EARTH = 6378137;
    double BIAD_RATE = 0.0066943800229007;
    double height = 0.0;
    double sum_dis = 0.0;
    double vehicle_length = 9.0;
    int mean_filter_num = 5;
    int vehicle_length_num = vehicle_length/d_s;
    p_x.clear();
    p_y.clear();
    s.clear();

    double x_pre = 0.0;
    double y_pre = 0.0;
    s.append(sum_dis);

    for(int i = 0; i < dis_lon_main_path.size() - 1; i++)
    {
        ENInfo eninfo;
        double P_radius = M_EARTH/sqrt(1-BIAD_RATE*sin(dis_lat_main_path[i]/180*M_PI)*sin(dis_lat_main_path[i]/180*M_PI));//卯酉半径
        double x_temp = (P_radius+height)*cos(dis_lat_main_path[i]/180*M_PI)*cos(dis_lon_main_path[i]/180*M_PI);
        double y_temp = (P_radius+height)*cos(dis_lat_main_path[i]/180*M_PI)*sin(dis_lon_main_path[i]/180*M_PI);
        double z_temp = (P_radius*(1-BIAD_RATE)+height)*sin(dis_lat_main_path[i]/180*M_PI);

        if(eninfo.iHead < 90)
        {
            eninfo.theta = (90-eninfo.iHead)/180.0*M_PI;
        }
        else
        {
            eninfo.theta = (450-eninfo.iHead)/180.0*M_PI;
        }
        s.append(sum_dis);

        if(i==0)
        {
            psiz = M_PI*0.5 - dis_lat_main_path[i]/180*M_PI;
            psiy = M_PI*0.5 + dis_lon_main_path[i]/180*M_PI;
            x_I = x_temp;
            y_I = y_temp;
            z_I = z_temp;
            eninfo.xEN=0;
            eninfo.yEN=0;
            eninfo.zEN=0;
            eninfo.iCurvature=0;
            p_x.append(eninfo.xEN);
            p_y.append(eninfo.yEN);
            double dis_temp = sqrt(pow(eninfo.xEN - x_pre,2)+pow(eninfo.yEN - y_pre,2));
            sum_dis += dis_temp;
        }
        else
        {
            eninfo.xEN=cos(psiy)*(x_temp-x_I)+sin(psiy)*(y_temp-y_I);
            eninfo.yEN=-cos(psiz)*sin(psiy)*(x_temp-x_I)+cos(psiz)*cos(psiy)*(y_temp-y_I)+sin(psiz)*(z_temp-z_I);
            eninfo.zEN=sin(psiz)*sin(psiy)*(x_temp-x_I)-sin(psiz)*cos(psiy)*(y_temp-y_I)+cos(psiz)*(z_temp-z_I);
            p_x.append(eninfo.xEN);
            p_y.append(eninfo.yEN);
            double dis_temp = sqrt(pow(eninfo.xEN - x_pre,2) + pow(eninfo.yEN - y_pre,2));
            sum_dis += dis_temp;
        }

        double temp_front_alt = 0.0, temp_rear_alt = 0.0;
        int sum_counter = 0;
        for(int j = i; j< (dis_alt_main_path.size() - vehicle_length_num) && sum_counter < mean_filter_num; ++j)
        {
            temp_front_alt += dis_alt_main_path.at(j + vehicle_length_num);
            temp_rear_alt += dis_alt_main_path.at(j);
            sum_counter++;
        }

        temp_front_alt /= mean_filter_num;
        temp_rear_alt /= mean_filter_num;

        double pitch_bias = 0.0;
        pitch.push_back((temp_front_alt - temp_rear_alt)/vehicle_length*g + pitch_bias);
        q_pitch.append((temp_front_alt - temp_rear_alt)/vehicle_length*g + pitch_bias);

//      pitch.push_back(0.0);
//      q_pitch.append(0.0);

        x_pre = eninfo.xEN;
        y_pre = eninfo.yEN;
    }
}


QString MainWindow::TextStreamRead(QString path)
{
    QString rData;
    QFile file(path);
    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QByteArray line = file.readAll();
        rData=line;
    }
    file.close();
    return rData;
}

void MainWindow::on_pb_load_main_path_clicked()
{
    initMainPathFiles();
    TransToENU();

    QPen drawPen;
    drawPen.setColor(Qt::red);
    drawPen.setWidth(4);
    ui->qCustomPlot->setInteraction(QCP::iRangeDrag, true); //鼠标单击拖动
    ui->qCustomPlot->setInteraction(QCP::iRangeZoom, true); //滚轮滑动缩放
    ui->qCustomPlot->clearGraphs();

    QCPGraph * curGraph = ui->qCustomPlot->addGraph();
    curGraph->setPen(drawPen);
    curGraph->setLineStyle(QCPGraph::lsNone);
    curGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    curGraph->setData(p_x, p_y);
    ui->qCustomPlot->legend->setVisible(true);
    ui->qCustomPlot->graph(0)->setName("矿区路径");

    //设置坐标轴标签名称
    ui->qCustomPlot->xAxis->setLabel("path_x");
    ui->qCustomPlot->yAxis->setLabel("path_y");

    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->qCustomPlot->xAxis->setRange(-100,1000);
    ui->qCustomPlot->yAxis->setRange(-100,1000);
    ui->qCustomPlot->replot();
    history_result_clean();
}

void MainWindow::history_result_clean()
{
    t_s.clear();
    k_s.clear();
    a_s.clear();
    jerk_s.clear();
    curve_s.clear();
    tra_r_s.clear();
    qp_s_x_.clear();
    qp_s_dx_.clear();
    qp_s_ddx_.clear();
    qp_s_jerk_t.clear();
}

void MainWindow::on_pb_plan_speed_clicked()
{

    /*-------------------------speed plot-------------------------*/
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->profile->clearGraphs();
    ui->profile_2->clearGraphs();
    ui->profile_3->clearGraphs();

    ui->profile->addGraph();
    ui->profile->setInteraction(QCP::iRangeDrag, true); //鼠标单击拖动
    ui->profile->setInteraction(QCP::iRangeZoom, true); //滚轮滑动缩放
    ui->profile->legend->setVisible(true);

    task_speed_plan(); //task desire speed setting
    curve_speed_plan();  //curve limitation maximum speed calculation
    osqpcal();// Interface to find the optimal speed profile

//    ui->profile->graph(0)->setData(s, k_s);
//    ui->profile->graph(0)->setName("矿区作业期望速度");

//    ui->profile->addGraph();
//    ui->profile->graph(1)->setPen(QPen(Qt::red));
//    ui->profile->graph(1)->setData(s, k_s);
//    ui->profile->graph(1)->setName("曲率约束下速度限制");

//    ui->profile->addGraph();
    ui->profile->graph(0)->setPen(QPen(Qt::darkGreen));
    ui->profile->graph(0)->setData(s, qp_s_x_);
    ui->profile->graph(0)->setName("最优速度规划结果");

    ui->profile->addGraph();
    ui->profile->graph(1)->setPen(QPen(Qt::darkBlue));
    ui->profile->graph(1)->setData(s, manual_s);
    ui->profile->graph(1)->setName("log跑车速度曲线");

//    ui->profile->addGraph();
//    ui->profile->graph(4)->setPen(QPen(Qt::darkYellow));
//    ui->profile->graph(4)->setData(s, manual_design_spd);
//    ui->profile->graph(4)->setName("人工设计期望 速度曲线");


    //设置坐标轴标签名称
    ui->profile->xAxis->setLabel("Distance");
    ui->profile->yAxis->setLabel("speed [m/s]");

    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->profile->xAxis->setRange(0.0,100);
    ui->profile->yAxis->setRange(0.0,11.0);
    ui->profile->replot();

    /*-------------------------curve plot-------------------------*/
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->profile_2->addGraph();
    ui->profile_2->setInteraction(QCP::iRangeDrag, true); //鼠标单击拖动
    ui->profile_2->setInteraction(QCP::iRangeZoom, true); //滚轮滑动缩放
    ui->profile_2->graph(0)->setData(s,curve_s);
    ui->profile_2->legend->setVisible(true);
    ui->profile_2->graph(0)->setName("曲率");

    //设置坐标轴标签名称
    ui->profile_2->xAxis->setLabel("Distance");
    ui->profile_2->yAxis->setLabel("curve");
    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->profile_2->xAxis->setRange(0.0,100);
    ui->profile_2->yAxis->setRange(-0.15,0.15);
    ui->profile_2->replot();

    /*-------------------------QP-------------------------*/
    // 设置图例可见
    ui->profile_3->legend->setVisible(true);
    //向绘图区域QCustomPlot(从widget提升来的)添加一条曲线
    ui->profile_3->addGraph();
    ui->profile_3->setInteraction(QCP::iRangeDrag, true); //鼠标单击拖动
    ui->profile_3->setInteraction(QCP::iRangeZoom, true); //滚轮滑动缩放
    ui->profile_3->graph(0)->setPen(QPen(Qt::blue));
    ui->profile_3->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDiamond, 2));
    ui->profile_3->graph(0)->setData(s,qp_s_dx_);
    ui->profile_3->graph(0)->setName("加速度 m/s^2");

    ui->profile_3->addGraph();
    ui->profile_3->graph(1)->setPen(QPen(Qt::red));
    ui->profile_3->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 2));
    ui->profile_3->graph(1)->setData(s,qp_s_ddx_);
    ui->profile_3->graph(1)->setName("加加速度 d_a/ds");

    ui->profile_3->addGraph();
    ui->profile_3->graph(2)->setPen(QPen(Qt::darkGreen));
    ui->profile_3->graph(2)->setData(s,qp_s_jerk_t);
    ui->profile_3->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangle, 2));
    ui->profile_3->graph(2)->setName("加速度变化率 m/s^3");

    ui->profile_3->addGraph();
    ui->profile_3->graph(3)->setPen(QPen(Qt::darkMagenta));
    ui->profile_3->graph(3)->setData(s,q_pitch);
    ui->profile_3->graph(3)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssStar, 2));
    ui->profile_3->graph(3)->setName("坡度加速度 m/s^2");

    //设置坐标轴标签名称
    ui->profile_3->xAxis->setLabel("Distance");
    ui->profile_3->yAxis->setLabel("Control Input");
    //设置坐标轴显示范围,否则我们只能看到默认的范围
    ui->profile_3->xAxis->setRange(0.0,100.0);
    ui->profile_3->yAxis->setRange(-1.0,1.0);
    ui->profile_3->replot();
}


//task speed limitation
void MainWindow::task_speed_plan()
{
   t_s.clear();

   for(int i = 0; i < s.size(); ++i)
   {
        t_s.append(task_speed);
   }
}


//curvature speed limitation
void MainWindow::curve_speed_plan()
{
   double curve_speed = 0;
   double rollover_speed = 0;
   double sideslip_speed = 0;
   k_s.clear();
   curve_s.clear();
   tra_r_s.clear();
   Matrix<double,3,3> M;
   Matrix<double,3,3> M_1;
   Matrix<double,3,1> X;
   Matrix<double,3,1> Y;
   Matrix<double,3,1> A;
   Matrix<double,3,1> B;

    ofstream curve_output;
    ofstream limitedspeed_output;

    curve_output.open("curve_output.txt");
    limitedspeed_output.open("limitedspeed_output.txt");
    curve_output << "===========curve_speed===========" << endl;
    limitedspeed_output << "===========curvature==========="  << endl;

   for(int i = 3; i < s.size()-4; ++i)
   {
       //(x,y) of three adjacent points
       x1 = p_x.at(i-3);
       x2 = p_x.at(i);
       x3 = p_x.at(i+3);
       y1 = p_y.at(i-3);
       y2 = p_y.at(i);
       y3 = p_y.at(i+3);

       //distance between two points
       ta = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
       tb = sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2));
       //coefficient 矩阵赋值
       M(0,0)=1; M(0,1)=-ta; M(0,2)=ta*ta;
       M(1,0)=1; M(1,1)=0;   M(1,2)=0;
       M(2,0)=1; M(2,1)=tb;  M(2,2)=tb*tb;

       //(x,y) 矩阵赋值
       X(0,0)=x1; X(1,0)=x2; X(2,0)=x3;
       Y(0,0)=y1; Y(1,0)=y2; Y(2,0)=y3;

       // caluate inverse of M
       M_1=M.inverse();
       // caluate A & B
       A=M_1*X;
       B=M_1*Y;

       // caluate curvature and R
       curvature = 2*(A(2,0)*B(1,0)-B(2,0)*A(1,0))/qPow((A(1,0)*A(1,0)+B(1,0)*B(1,0)),1.5);
       tra_r = abs(1/curvature);
       //std::cout<<"qvlv:"<<curvature<<"   banjing:"<<tra_r<<std::endl;

       //侧翻速度约束
       rollover_speed = sqrt((b/2*h)*g*tra_r);

       //侧滑速度约束，这里综合人工设计速度
       double temp_task_speed = manual_design_spd.at(i);
       sideslip_speed = std::min(temp_task_speed, sqrt(fai*g*tra_r));   //m/s

       //commonly
       curve_speed = sideslip_speed;
       curve_speed *= k_robot_impact;

       if((curve_speed>10) || (std::isnan(curve_speed)))     // 防止出现曲率突变问题，增加一项校验
           curve_speed = 10;

       k_s.append(curve_speed);
       curve_s.append(curvature);
       curve_output << curve_speed << endl;
       limitedspeed_output << curvature << endl;
   }

   curve_output.close();
   limitedspeed_output.close();
}

//acceleration limitation
void MainWindow::acc_speed_plan()
{
   double acc_speed = 15.0;
   a_s.clear();

   for(int i = 0; i < s.size(); ++i)
   {
        a_s.append(acc_speed);
   }
}


//jerk limitation
void MainWindow::jerk_speed_plan()
{
   double jerk_speed = 15.0;
   jerk_s.clear();

   for(int i = 0; i < s.size(); ++i)
   {
        jerk_s.append(jerk_speed);
   }

}

void MainWindow::osqpcal()
{
    std::cout<<"OSQP RUNNING..."<<std::endl;
    PathBoundaryInfo bound_info;

    //boundary
    bound_info.set_delta_s(d_s); // setting delta s, current 0.2m for each path point
    PathBoundary bound_result;
    k_s = manual_design_spd;
    for (size_t i = 0; i < k_s.size(); ++i)
    {
        bound_result.emplace_back(std::make_pair(8.0, qPow(manual_design_spd[i]*1.2,2))); // (m/s)^2 loading the curve limitaion speed calculation result
    }

    bound_info.set_boundary(bound_result);
    std::array<double, 3> init_state = {25.0, 0.0, 0.0};// setting the initial speed, current speed 4m/s
    OSQPProblem prob(k_s.size(), bound_info.delta_s(), init_state); // setting the optimized path point num and inital state
    std::array<double, 3> end_state = {25.0, 0.0, 0.0}; //setting the final speed, in this situation, the end state is not used.
    std::vector<double> ref_vec(k_s.size(), task_speed*task_speed);  // set the reference speed 5m/s

    for(size_t i=0; i < k_s.size();++i)
    {
        ref_vec.at(i) = manual_design_spd[i]*manual_design_spd[i]*1.2;
    }

    double vehicle_max_acc = 1.0;
    double vehicle_min_acc = -1.0;
    double vehicle_desire_min_acc = -1.5;
    double vehicle_desire_max_acc = 1.5;

    prob.set_weight_x(1.0);
    prob.set_weight_dx(100.0);
    prob.set_weight_ddx(1000.0);
    prob.set_weight_dddx(1000.0);
    prob.set_end_state_ref({{1.0, 1.0, 1.0}}, end_state);
    prob.set_x_ref(0.1,ref_vec);// set the desire devitation term weight
    prob.set_scale_factor({1.0, 1.0, 1.0});
    prob.set_x_bounds(bound_info.boundary()); // set the speed bound
    prob.set_dx_bounds(vehicle_min_acc, vehicle_max_acc); // set the acceleraion bound
    prob.set_ddx_bounds(-2.0, 2.0); // set the jerk bound
    prob.set_pitch_set(pitch); // set the gravity acceleration based the pitch

    std::vector<std::pair<double, double>> acc_bounds_set;
    for(size_t i=0; i < pitch.size();++i)
    {
         double temp_min_acc = std::max(vehicle_min_acc, vehicle_desire_min_acc - pitch[i]);
         double temp_max_acc = std::min(vehicle_max_acc, vehicle_desire_max_acc - pitch[i]);
         if(temp_max_acc - temp_min_acc >0.1)
         {
              acc_bounds_set.push_back(std::make_pair(temp_min_acc,temp_max_acc));
         } else
         {
             acc_bounds_set.push_back(std::make_pair(vehicle_min_acc, vehicle_max_acc));
         }
    }

    if(pitch.size() >= k_s.size())
    {
        prob.set_dx_bounds_set(acc_bounds_set);
        std::cout<<"Finish the desire acc setting"<<std::endl;
    }

    ofstream mycout;
    time_t nowtime = time(NULL);
    struct tm *p;
    p = gmtime(&nowtime);
    char timeinfo[256] = {0};
    sprintf(timeinfo,"%d-%d-%d %d-%02d-osqp-traj",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min);
    mycout.open(timeinfo);
    QTime time;

    ofstream result_output;
    result_output.open("result_output.txt");
    result_output <<"===========result_speed==========="  << endl;


//    double emergy_min = 10000;
//    double emergy_max = 8000;
//    double max_w_1,max_w_2,max_w_3,max_w_4;
//    double min_w_1,min_w_2,min_w_3,min_w_4;

//    for(int i=40; i<100;i+=10)
//    {
//        std::cout<<"run i: "<< i <<std::endl;
//        for(int j=1;j<102;j+=20)
//        {
//            std::cout<<"run"<<std::endl;

//            for(int k=1;k<102;k+=20)
//            {
//                for(int l=1;l<102;l+=40)
//                {
//                    prob.set_weight_x(i*0.1);
//                    prob.set_weight_dx(j);
//                    prob.set_weight_ddx(k);
//                    prob.set_weight_dddx(l);
//                    if(prob.Optimize(100))
//                    {
//                        double acc_sum = 0.0;
//                        for (int p = 0; p < k_s.size(); ++p)
//                        {
//                            acc_sum+=fabs(prob.dx_.at(p));
//                        }

//                        if(acc_sum >emergy_max && acc_sum < 100000)
//                        {
//                            emergy_max = acc_sum;
//                            std::cout<<"max energy: "<< emergy_max<<"i: "<<i*0.1<<" j: "<<j<<" k: "<<k<<" l: "<<l<<std::endl;
//                            max_w_1= 0.1*i;
//                            max_w_2= j;
//                            max_w_3= k;
//                            max_w_4= l;
//                        }

//                        if(acc_sum <emergy_min && acc_sum > -100000)
//                        {
//                            emergy_min = acc_sum;
//                            std::cout<<"min energy: "<< emergy_min<<"i: "<<i*0.1<<" j: "<<j<<" k: "<<k<<" l: "<<l<<std::endl;
//                            min_w_1= 0.1*i;
//                            min_w_2= j;
//                            min_w_3= k;
//                            min_w_4= l;
//                        }
//                    }
//                }
//            }
//        }
//    }

    // start to optimize
    time.start();

    if(prob.Optimize(1000))
    {
        std::cout<<"Optimize successful!!"<<std::endl;
        double acc_sum = 0.0;

        qDebug()<<"timeuse: "<<time.elapsed()/1000.0<<"s";
       for (int i = 0; i < k_s.size()-1; ++i)
       {
           if(i==0)
          {
               mycout<<"x: "<<prob.x_.at(i)<<" dx: "<<prob.dx_.at(i)<<" ddx: "<<prob.ddx_.at(i)<<" left_edge: "<<bound_info.boundary_.at(i).second<<
                       " right_edge: "<<bound_info.boundary_.at(i).first<<std::endl;
               qp_s_x_.append(sqrt(prob.x_.at(i)));
               qp_s_dx_.append(prob.dx_.at(i));
               qp_s_ddx_.append(prob.ddx_.at(i));
               jerk_t = 0;
               qp_s_jerk_t.append(jerk_t);
               result_output <<sqrt(prob.x_.at(i)) << endl;
          }
          else
          {
                mycout<<"x: "<<prob.x_.at(i)<<" dx: "<<prob.dx_.at(i)<<" ddx: "<<prob.ddx_.at(i)<<" left_edge: "<<bound_info.boundary_.at(i).second<<
                        " right_edge: "<<bound_info.boundary_.at(i).first<<std::endl;
                qp_s_x_.append(sqrt(prob.x_.at(i)));
                acc_sum+=fabs(prob.dx_.at(i));
                qp_s_dx_.append(prob.dx_.at(i));
                qp_s_ddx_.append(prob.ddx_.at(i));
                a_t = prob.dx_.at(i-1);
                a_t_ = prob.dx_.at(i);
                ts = 0.2/sqrt(prob.x_.at(i-1));  //t=delta_s/v  (s)
                jerk_t = (a_t_- a_t)/ts;
                qp_s_jerk_t.append(jerk_t);
                result_output <<sqrt(prob.x_.at(i)) << endl;
          }
      }

       std::cout<<"accumulate energy: "<<acc_sum<<std::endl;

    }else
    {
      std::cout<<"Optimize failed!!"<<std::endl;
    }

    qDebug()<<"timeuse: "<<time.elapsed()/1000.0<<"s";
    std::vector<int> match_index_set;
    int pre_index = 0;

    for(int i = 0; i < k_s.size()-1; ++i)
    {
        double distance = 1000.0;
        int nearest_index = 0;

        for(int j = pre_index; j < dis_lon_log_path.size()-1;++j)
        {
            double lon = dis_lon_main_path.at(i);  //与原生路径进行匹配
            double lat = dis_lat_main_path.at(i);
//            double pre_lon =  dis_lon_main_path.at(i+1);
//            double pre_lat =  dis_lat_main_path.at(i+1);
            double temp_dis = getGPSDistance(lat,lon, dis_lat_log_path[j], dis_lon_log_path[j]);
//            double pre_dis = getGPSDistance(pre_lat, pre_lon, dis_lat_log_path[j], dis_lon_log_path[j]);

            if(temp_dis < distance)
            {
                distance = temp_dis;
                nearest_index = j;
            }
        }

        match_index_set.push_back(nearest_index);
    }

    manual_s = qp_s_x_;
    for(int i = 1; i < qp_s_x_.size(); ++i)
    {
        int match_index = match_index_set[i];
        if(match_index != -1 )
        {
            manual_s[i] = dis_spd_log_path[match_index];
        }
        else
        {
            manual_s[i] = manual_s[i-1];
        }
    }


    eval_traj(manual_s, qp_s_x_, manual_design_spd);

    result_output.close();
    std::cout<<"finish speed planning"<<std::endl;
}


// 规划结果与代价评估函数
void MainWindow::eval_traj(const QVector<double> &manual_s, const QVector<double> &t_s, const QVector<double> &manual_design_spd)
{
    // count the path point exceed the maximum speed

    int manual_break, auto_break = 0;

    for(int i = 0 ; i < manual_s.size() ;++i)
    {
        if(manual_s.at(i) > manual_design_spd.at(i))
            manual_break++;

        if(t_s.at(i) > manual_design_spd.at(i))
            auto_break++;
    }

    std::cout<<"人工驾驶超速 "<<manual_break<<" 个点"<<std::endl;
    std::cout<<"自动驾驶超速 "<<auto_break<<" 个点"<<std::endl;

    double delta_s = 0.2;
    double manual_maximum_acc = 0.0;
    double auto_maximum_acc = 0.0;

    double manual_travel_time = 0.0;
    double auto_travel_time = 0.0;

    double manual_energy = 0.0;
    double auto_energy = 0.0;


    for(int i = 0 ; i < manual_s.size()-1 ;++i)
    {
        double acc = (manual_s.at(i+1)*manual_s.at(i+1)-manual_s.at(i)*manual_s.at(i))/(2*delta_s);
        if(fabs(acc) < manual_maximum_acc)
            manual_maximum_acc = fabs(acc);

        manual_travel_time += delta_s / manual_s.at(i);
        manual_energy += fabs(acc-q_pitch.at(i));
    }

    for(int i = 0 ; i < t_s.size()-1 ;++i)
    {
        double acc = (t_s.at(i+1)*t_s.at(i+1)-t_s.at(i)*t_s.at(i))/(2*delta_s);
        if(fabs(acc) < auto_maximum_acc)
            auto_maximum_acc = fabs(acc-q_pitch.at(i));

        auto_travel_time+= delta_s / t_s.at(i);
        auto_energy += fabs(acc);
    }

    std::cout<<"人工驾驶最大加速度: "<<manual_maximum_acc<<std::endl;
    std::cout<<"自动驾驶最大加速度: "<<auto_maximum_acc<<std::endl;

    std::cout<<"人工驾驶行驶时间: "<<manual_travel_time<<std::endl;
    std::cout<<"自动驾驶行驶时间: "<<auto_travel_time<<std::endl;

    std::cout<<"人工驾驶行驶能耗: "<<manual_energy<<std::endl;
    std::cout<<"自动驾驶行驶能耗: "<<auto_energy<<std::endl;
}


MainWindow::~MainWindow()
{
    delete ui;
}
