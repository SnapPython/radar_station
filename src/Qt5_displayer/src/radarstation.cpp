#include "radarstation.h"
#include "ui_radarstation.h"
#include "pnpwidget.h"
#include "ui_pnpwidget.h"
#include "smallmap.h"
#include <QFileDialog>
#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QRadioButton>

radarStation::radarStation(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::radarStation)
{
    ui->setupUi(this);
//    this->resize(1600,1400);
//    ui->centralwidget->resize(1600,1400);
    //ui->centralwidget->installEventFilter(this);
    //QString image_path = QFileDialog::getOpenFileName(this, "打开图片", ".", "Images (*.png *.xpm *.jpg)");
//    QString image_path = "C:/Users/zyb/Desktop/Robotmaster/pitures/map.png";
//    ui->map->setPixmap(QPixmap(image_path));
    ui->map->mapx = ui->map->x();
    ui->map->mapy = ui->map->y();
    ui->map->mapcols = ui->map->width();
    ui->map->maprows = ui->map->height();
    ui->allWidget->setCurrentWidget(ui->mapWidget);

    init();
}

radarStation::~radarStation()
{
    delete ui;
}

void radarStation::init()
{
    connect(ui->pnpMode,SIGNAL(clicked()),this,SLOT(changeToPnpWidget()));
    connect(ui->mapMode,SIGNAL(clicked()),this,SLOT(changeToMapWidget()));
    connect(&this->qtnode,SIGNAL(updateFarImage()),this,SLOT(farImageUpdate()));
    connect(&this->qtnode,SIGNAL(updateDepthImage()),this,SLOT(depthImageUpdate()));
    connect(&this->qtnode,SIGNAL(updatePoints()),this,SLOT(pointsUpdate()));
    connect(ui->solvePnpWidget,SIGNAL(pnpFinished()),this,SLOT(publishPnpResult()));
}

void radarStation::mousePressEvent(QMouseEvent *event)
{
    //qDebug() << "x:" << event->x() << "y:" << event->y();
    /*if(event->x() >= mapx && event->x() <= mapx + mapcols)
    {
        if(event->y() >= mapy && event->y() <= mapy + maprows)
        {
            QPainter painter(ui->map);
            painter.setPen(QPen(Qt::black,2));
            painter.drawEllipse(event->pos(),5,5);
            ui->map->update();
        }
    }*/
    QString text = "x:";
    text = text + QString::number(event->x());
    text = text + " y:" + QString::number(event->y());
    mapMessageDisplay(text);
}

void radarStation::paintEvent(QPaintEvent *event)
{
    QPainter t(this);
    QPen pen;
    pen.setWidth(5);
    t.setPen(pen);
    t.drawEllipse(pos,5,5);
    event->accept();
}

void radarStation::mapMessageDisplay(QString text)
{
     QPlainTextEdit *messageEdit = ui->mapMessage;
     if(!messageEdit->isReadOnly())
     {
         messageEdit->setReadOnly(true);
     }
     //messageEdit->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
     if(messageEdit->toPlainText().size() > 1024 * 4)
     {
         messageEdit->clear();
     }
     //messageEdit->insertPlainText(text);
     messageEdit->appendPlainText(text);
     QScrollBar * scrollbar = messageEdit->verticalScrollBar();
     if(scrollbar)
     {
         scrollbar->setSliderPosition(scrollbar->maximum());
     }
}

void radarStation::changeToMapWidget()
{
    //qDebug() << "map";
    ui->allWidget->setCurrentWidget(ui->mapWidget);
}

void radarStation::changeToPnpWidget()
{
    //qDebug() << "pnp";
    ui->allWidget->setCurrentWidget(ui->solvePnpWidget);
}

void radarStation::farImageUpdate()
{
    qimage_mutex.lock();
    //mapMessageDisplay("farImageUpdate");
    if(ui->pnpMode->isChecked())
    {
        //cout << "pnpMode" << endl;
        ui->solvePnpWidget->cameraimage = QPixmap::fromImage(qtnode.far_qimage);
        ui->solvePnpWidget->ui->cameraImage->update();
    }
    else if(ui->mapMode->isChecked())
    {
        //cout << "mapMode" << endl;
        ui->imgHandle->setPixmap(QPixmap::fromImage(qtnode.far_qimage));
        ui->imgHandle->update();
    }
    qimage_mutex.unlock();
}

void radarStation::depthImageUpdate()
{
    qimage_mutex.lock();
    //mapMessageDisplay("depthImageUpdate");
    if(ui->mapMode->isChecked())
    {
        //cout << "绘画深度图" << endl;
        ui->radarDepth->setPixmap(QPixmap::fromImage(qtnode.depth_qimage));
        ui->radarDepth->update();
    }
    qimage_mutex.unlock();
}

void radarStation::publishPnpResult()
{
    std_msgs::msg::Float32MultiArray pnp_result;
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(0,0));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(0,1));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(0,2));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(1,0));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(1,1));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(1,2));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(2,0));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(2,1));
    pnp_result.data.push_back(ui->solvePnpWidget->far_R.at<double>(2,2));
    pnp_result.data.push_back(ui->solvePnpWidget->far_T.at<double>(0,0));
    pnp_result.data.push_back(ui->solvePnpWidget->far_T.at<double>(1,0));
    pnp_result.data.push_back(ui->solvePnpWidget->far_T.at<double>(2,0));
    qtnode.pnp_pub_->publish(pnp_result);
    cout << "发送pnp结果" << endl;
}

void radarStation::pointsUpdate()
{
    float object_width = 11.5;
    float object_height = 8;
    my_msgss::msg::Points qpoints = qtnode.world_qpoints;
    float width = ui->map->width() * ui->map->scaleValue;
    float height = ui->map->height() * ui->map->scaleValue;
    mapPos pos;
    for(int i = 0;i<qpoints.data.size();i++)
    {
        pos.x = qpoints.data[i].x / object_width * width;
        pos.y = height - (qpoints.data[i].y / object_height * height);
        pos.x = pos.x + ui->map->drawPos.x();
        pos.y = pos.y + ui->map->drawPos.y();
        pos.id = qpoints.data[i].id;
        ui->map->mapPoints.push_back(pos);
        QString pointText = QString::number(pos.id) + "号机器人相对于初始小地图的坐标为 x:" 
                            + QString::number((pos.x - ui->map->drawPos.x()) / ui->map->scaleValue) +" y:"
                            + QString::number((pos.y - ui->map->drawPos.y()) / ui->map->scaleValue);
        mapMessageDisplay(pointText);
    }
    ui->map->update();
}

