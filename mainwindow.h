#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qcustomplot.h>
#include <data_struct.h>
#include <iostream>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QVector<double> dis_lat_main_path,dis_lon_main_path, dis_alt_main_path;
    QVector<double> dis_lat_log_path,dis_lon_log_path, dis_spd_log_path;

    std::vector<GPSInfo> main_path_gps;
    QString TextStreamRead(QString path);
    void initMainPathFiles();
    void initLogPathFiles();
    QVector<double> p_x, p_y, s;
    double x1,x2,x3,y1,y2,y3;
    double ta,tb;
    double curvature;
    double tra_r;
    double d_s = 0.2; //distance between two adjacent points
    double b=9.40, h = 2.61, fai=0.5, g=9.8, l = 9.0; // b:the weight of vehicle, h:the height of vehicle, fai:friction coefficient of pavement
    double k_robot_impact = 1.0, p=0.3;
    double task_speed = 6.0;
    double ts,a_t,a_t_,jerk_t;
    QVector<double> t_s, k_s, a_s, jerk_s, curve_s, tra_r_s, qp_s_x_, qp_s_dx_, qp_s_ddx_,qp_s_jerk_t,q_pitch, manual_s, manual_design_spd;
    std::vector<double> pitch;

    void task_speed_plan();
    void jerk_speed_plan();
    void acc_speed_plan();
    void curve_speed_plan();
    void osqpcal();
    void TransToENU();
    void history_result_clean();
    void eval_traj(const QVector<double> &manual_s, const QVector<double> &t_s, const QVector<double> &manual_design_spd);

private slots:
    void on_pb_load_main_path_clicked();
    void on_pb_plan_speed_clicked();

};

#endif // MAINWINDOW_H
