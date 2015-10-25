#include "mainwindow.h"
#include "ui_mainwindow.h"
using namespace cv;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
QImage MainWindow::matToQImage(cv::Mat mat) {
    if(mat.channels() == 1) {                   // if grayscale image
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);     // declare and return a QImage
    } else if(mat.channels() == 3) {            // if 3 channel color image
        cv::cvtColor(mat, mat, CV_BGR2RGB);     // invert BGR to RGB
        return QImage((uchar*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);       // declare and return a QImage
    } else {
        qDebug() << "in openCVMatToQImage, image was not 1 channel or 3 channel, should never get here";
    }
    return QImage();        // return a blank QImage if the above did not work
}

void MainWindow::on_btnLeft_clicked()
{

    QString strFileName = QFileDialog::getOpenFileName();       // bring up open file dialog



    orginalL = cv::imread(strFileName.toStdString());        // open image

     QImage QorginalL = matToQImage(orginalL);         // convert original and Canny images to QImage
     ui->picL->setPixmap(QPixmap::fromImage(QorginalL));   // show original and Canny images on labels
}
