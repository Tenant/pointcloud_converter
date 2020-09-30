#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <QFileDialog>
#include <iostream>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "./include/VelodyneStreamer.h"
#include "./include/LiDARDataFileInterface.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->textEdit_show->document()->setMaximumBlockCount(14);

    connect(ui->btn_source_data, SIGNAL(clicked()), this, SLOT(slotSelectSourceData()));
    connect(ui->btn_destination, SIGNAL(clicked()), this, SLOT(slotSelectDestination()));
    connect(ui->btn_start, SIGNAL(clicked()), this, SLOT(slotStartProcessing()));
    connect(ui->btn_quit, SIGNAL(clicked()), qApp, SLOT(quit()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::slotSelectSourceData()
{
    _strSourcePath = QFileDialog::getOpenFileName(this, tr("Select source data files"), ".");
    _strSourceDir = QFileInfo(_strSourcePath).absolutePath();
    _strSourceName = QFileInfo(_strSourcePath).fileName().split(".")[0];
    ui->lineEdit_source_data->setText(_strSourcePath);
}

void MainWindow::slotSelectDestination()
{
    QString strDestination = QFileDialog::getExistingDirectory(this, tr("Select destination folder"), _strSourceDir);
    ui->lineEdit_destination->setText(strDestination);
}

void MainWindow::slotStartProcessing()
{
    int src_mode = ui->cmb_src_format->currentIndex();
    int dst_mode = ui->cmb_dst_format->currentIndex();

    if (src_mode == SRC_FMT::PCAP_HDL32 && dst_mode == DST_FMT::ROSBAG_POINTCLOUD2) {
        ui->textEdit_show->append("converte from PCAP_HDL32 to ROSBAG_POINTCLOUD2");

        VelodyneStreamer streamer(_strSourcePath.toStdString(), "pcap");
        streamer.setCalib("/home/sukie/32db.xml");

        LiDARData data(32, 2400);

        int frame= 0;
        QString strDstPath = _strSourceDir + "/" + _strSourceName + ".bag";
        rosbag::Bag bag(strDstPath.toStdString(), rosbag::bagmode::Write);
        while(streamer.nextFrame(data))
        {
            frame++;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (int line = 0; line != data.getLineNum(); line++)
            {
                int lineptnum = data[line].size();
                for (int col = 0; col < lineptnum; col++)
                {
                    auto x = data[line].getX(col);
                    auto y = data[line].getY(col);
                    auto z = data[line].getZ(col);
                    auto i = data[line].getI(col);
                    pcl::PointXYZI p;
                    p.x = x;
                    p.y = y;
                    p.z = z;
                    p.intensity = i;
                    cloud->push_back(p);
                }
        }
            ros::Time tt(data.timeStamp_tail / 1000000, data.timeStamp_tail % 1000000 * 1000);
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.frame_id = "map";
            bag.write("velodyne", tt, msg);
        }
        bag.close();
        ui->textEdit_show->append("conversion finished");
    }
    else if (src_mode == SRC_FMT::PCAP_HDL32 && dst_mode == DST_FMT::CSV_TIMESTAMP) {
        ui->textEdit_show->append("start convertion from PCAP_HDL32 to ROSBAG_CSV_TIMESTAMP");
        ui->textEdit_show->repaint();

        VelodyneStreamer streamer(_strSourcePath.toStdString(), "pcap");
        streamer.setCalib("/home/sukie/32db.xml");

        LiDARData data(32, 2400);

        int frame= 0;
        QString strDstPath = _strSourceDir + "/" + _strSourceName + ".csv";
        FILE *fpTime = fopen(strDstPath.toStdString().c_str(), "w");
        while(streamer.nextFrame(data))
        {
            frame++;
            fprintf(fpTime, "%ld\n", data.timeStamp_tail);
        }
        fclose(fpTime);
        ui->textEdit_show->append("conversion finished");
    }

}

