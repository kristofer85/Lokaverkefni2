#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<opencv/cv.h>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

#include <QMainWindow>
#include<QFileDialog>
#include<QtCore>
using namespace cv;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Mat orginalL;

private slots:
    QImage matToQImage(Mat mat);
    void on_btnLeft_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
