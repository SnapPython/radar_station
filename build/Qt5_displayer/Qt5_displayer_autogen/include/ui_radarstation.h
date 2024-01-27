/********************************************************************************
** Form generated from reading UI file 'radarstation.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RADARSTATION_H
#define UI_RADARSTATION_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QWidget>
#include <pnpwidget.h>
#include <smallmap.h>

QT_BEGIN_NAMESPACE

class Ui_radarStation
{
public:
    QWidget *centralwidget;
    QRadioButton *pnpMode;
    QRadioButton *mapMode;
    QStackedWidget *allWidget;
    QWidget *mapWidget;
    QLabel *imgHandle;
    smallMap *map;
    QPlainTextEdit *mapMessage;
    QLabel *radarDepth;
    pnpWidget *solvePnpWidget;

    void setupUi(QMainWindow *radarStation)
    {
        if (radarStation->objectName().isEmpty())
            radarStation->setObjectName(QString::fromUtf8("radarStation"));
        radarStation->resize(1600, 1400);
        centralwidget = new QWidget(radarStation);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        centralwidget->setAutoFillBackground(false);
        pnpMode = new QRadioButton(centralwidget);
        pnpMode->setObjectName(QString::fromUtf8("pnpMode"));
        pnpMode->setGeometry(QRect(430, 20, 101, 51));
        pnpMode->setChecked(false);
        mapMode = new QRadioButton(centralwidget);
        mapMode->setObjectName(QString::fromUtf8("mapMode"));
        mapMode->setGeometry(QRect(290, 20, 141, 51));
        mapMode->setCheckable(true);
        mapMode->setChecked(true);
        mapMode->setAutoRepeat(false);
        allWidget = new QStackedWidget(centralwidget);
        allWidget->setObjectName(QString::fromUtf8("allWidget"));
        allWidget->setGeometry(QRect(0, 0, 1600, 1400));
        mapWidget = new QWidget();
        mapWidget->setObjectName(QString::fromUtf8("mapWidget"));
        imgHandle = new QLabel(mapWidget);
        imgHandle->setObjectName(QString::fromUtf8("imgHandle"));
        imgHandle->setEnabled(true);
        imgHandle->setGeometry(QRect(1150, 170, 391, 311));
        imgHandle->setAutoFillBackground(true);
        imgHandle->setPixmap(QPixmap(QString::fromUtf8("../../../Pictures/Screenshots/1.png")));
        imgHandle->setScaledContents(true);
        map = new smallMap(mapWidget);
        map->setObjectName(QString::fromUtf8("map"));
        map->setGeometry(QRect(0, 80, 1041, 531));
        map->setAutoFillBackground(true);
        map->setPixmap(QPixmap(QString::fromUtf8("../../../Pictures/Screenshots/1.png")));
        map->setScaledContents(true);
        map->setWordWrap(false);
        mapMessage = new QPlainTextEdit(mapWidget);
        mapMessage->setObjectName(QString::fromUtf8("mapMessage"));
        mapMessage->setGeometry(QRect(0, 610, 781, 401));
        mapMessage->setLineWrapMode(QPlainTextEdit::WidgetWidth);
        mapMessage->setReadOnly(true);
        mapMessage->setOverwriteMode(false);
        radarDepth = new QLabel(mapWidget);
        radarDepth->setObjectName(QString::fromUtf8("radarDepth"));
        radarDepth->setGeometry(QRect(1140, 630, 391, 311));
        radarDepth->setAutoFillBackground(true);
        radarDepth->setPixmap(QPixmap(QString::fromUtf8("../../../Pictures/Screenshots/1.png")));
        radarDepth->setScaledContents(true);
        allWidget->addWidget(mapWidget);
        radarDepth->raise();
        mapMessage->raise();
        map->raise();
        imgHandle->raise();
        solvePnpWidget = new pnpWidget();
        solvePnpWidget->setObjectName(QString::fromUtf8("solvePnpWidget"));
        allWidget->addWidget(solvePnpWidget);
        radarStation->setCentralWidget(centralwidget);
        allWidget->raise();
        pnpMode->raise();
        mapMode->raise();

        retranslateUi(radarStation);

        allWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(radarStation);
    } // setupUi

    void retranslateUi(QMainWindow *radarStation)
    {
        radarStation->setWindowTitle(QCoreApplication::translate("radarStation", "radarStation", nullptr));
        pnpMode->setText(QCoreApplication::translate("radarStation", "PnP\346\250\241\345\274\217", nullptr));
        mapMode->setText(QCoreApplication::translate("radarStation", "\345\260\217\345\234\260\345\233\276\346\250\241\345\274\217", nullptr));
        imgHandle->setText(QString());
        map->setText(QString());
        radarDepth->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class radarStation: public Ui_radarStation {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RADARSTATION_H
