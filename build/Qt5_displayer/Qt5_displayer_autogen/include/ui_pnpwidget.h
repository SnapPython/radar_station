/********************************************************************************
** Form generated from reading UI file 'pnpwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PNPWIDGET_H
#define UI_PNPWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_pnpWidget
{
public:
    QLabel *cameraImage;
    QPushButton *beginPoint;
    QPushButton *restartPoint;
    QPushButton *beginPnp;
    QPlainTextEdit *message;

    void setupUi(QWidget *pnpWidget)
    {
        if (pnpWidget->objectName().isEmpty())
            pnpWidget->setObjectName(QString::fromUtf8("pnpWidget"));
        pnpWidget->resize(1600, 1400);
        cameraImage = new QLabel(pnpWidget);
        cameraImage->setObjectName(QString::fromUtf8("cameraImage"));
        cameraImage->setGeometry(QRect(110, 80, 640, 480));
        cameraImage->setFocusPolicy(Qt::NoFocus);
        cameraImage->setToolTipDuration(-1);
        cameraImage->setAutoFillBackground(true);
        cameraImage->setPixmap(QPixmap(QString::fromUtf8("../../../Pictures/Screenshots/1.png")));
        cameraImage->setScaledContents(true);
        cameraImage->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        beginPoint = new QPushButton(pnpWidget);
        beginPoint->setObjectName(QString::fromUtf8("beginPoint"));
        beginPoint->setGeometry(QRect(830, 80, 121, 31));
        restartPoint = new QPushButton(pnpWidget);
        restartPoint->setObjectName(QString::fromUtf8("restartPoint"));
        restartPoint->setGeometry(QRect(830, 180, 121, 31));
        beginPnp = new QPushButton(pnpWidget);
        beginPnp->setObjectName(QString::fromUtf8("beginPnp"));
        beginPnp->setGeometry(QRect(830, 280, 121, 41));
        message = new QPlainTextEdit(pnpWidget);
        message->setObjectName(QString::fromUtf8("message"));
        message->setGeometry(QRect(110, 590, 1401, 391));
        message->setReadOnly(true);
        beginPoint->raise();
        restartPoint->raise();
        beginPnp->raise();
        cameraImage->raise();
        message->raise();

        retranslateUi(pnpWidget);

        QMetaObject::connectSlotsByName(pnpWidget);
    } // setupUi

    void retranslateUi(QWidget *pnpWidget)
    {
        pnpWidget->setWindowTitle(QCoreApplication::translate("pnpWidget", "Form", nullptr));
        cameraImage->setText(QString());
        beginPoint->setText(QCoreApplication::translate("pnpWidget", "\345\274\200\345\247\213\346\240\207\347\202\271", nullptr));
        restartPoint->setText(QCoreApplication::translate("pnpWidget", "\351\207\215\346\226\260\346\240\207\347\202\271", nullptr));
        beginPnp->setText(QCoreApplication::translate("pnpWidget", "\345\274\200\345\247\213pnp\347\273\223\347\256\227", nullptr));
    } // retranslateUi

};

namespace Ui {
    class pnpWidget: public Ui_pnpWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PNPWIDGET_H
