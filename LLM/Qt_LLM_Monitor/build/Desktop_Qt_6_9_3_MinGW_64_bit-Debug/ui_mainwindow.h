/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.9.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGraphicsView *g;
    QLabel *tempLabel;
    QLabel *vibLabel;
    QPushButton *summarizeButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        g = new QGraphicsView(centralwidget);
        g->setObjectName("g");
        g->setGeometry(QRect(170, -10, 461, 491));
        tempLabel = new QLabel(centralwidget);
        tempLabel->setObjectName("tempLabel");
        tempLabel->setGeometry(QRect(230, 100, 141, 41));
        QFont font;
        font.setPointSize(20);
        tempLabel->setFont(font);
        vibLabel = new QLabel(centralwidget);
        vibLabel->setObjectName("vibLabel");
        vibLabel->setGeometry(QRect(390, 100, 141, 41));
        vibLabel->setFont(font);
        summarizeButton = new QPushButton(centralwidget);
        summarizeButton->setObjectName("summarizeButton");
        summarizeButton->setGeometry(QRect(270, 180, 181, 51));
        summarizeButton->setFont(font);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 800, 18));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        tempLabel->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        vibLabel->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        summarizeButton->setText(QCoreApplication::translate("MainWindow", "\354\203\201\355\203\234\354\232\224\354\225\275", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
