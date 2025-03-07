/********************************************************************************
** Form generated from reading UI file 'base_placement_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASE_PLACEMENT_WIDGET_H
#define UI_BASE_PLACEMENT_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_BasePlacementWidget
{
public:
    QGridLayout *gridLayout;
    QProgressBar *progressBar;
    QTabWidget *tabWidget;
    QWidget *tab_1;
    QGridLayout *gridLayout_2;
    QLabel *label;
    QPushButton *btn_ClearAllPoints;
    QPushButton *btn_SavePath;
    QPushButton *btn_LoadPath;
    QTreeView *treeView;
    QSpacerItem *horizontalSpacer;
    QGridLayout *gridLayout_3;
    QPushButton *btnRemovePoint;
    QSpacerItem *verticalSpacer;
    QGroupBox *groupBox_3;
    QComboBox *combo_robotModel;
    QLineEdit *txtPointName;
    QLabel *label_3;
    QGroupBox *newPointLayout;
    QFormLayout *FormLayoutObjectPose_2;
    QGroupBox *groupBox;
    QLabel *label_2;
    QLabel *xLabel;
    QLineEdit *LineEditX;
    QLabel *yLabel;
    QLineEdit *LineEditY;
    QLabel *zLabel;
    QLineEdit *LineEditZ;
    QLabel *rxLabel;
    QLineEdit *LineEditRx;
    QLabel *ryLabel;
    QLineEdit *LineEditRy;
    QLabel *rzLabel;
    QLineEdit *LineEditRz;
    QPushButton *btnAddPoint;
    QSpacerItem *verticalSpacer_4;
    QWidget *tab_2;
    QPushButton *targetPoint;
    QGroupBox *groupBox_2;
    QLabel *label_5;
    QLineEdit *lnEdit_BaseLocSize;
    QComboBox *combo_planGroup;
    QLabel *label_6;
    QLineEdit *lnEdit_SpSize;
    QLabel *label_8;
    QLabel *lbl_placeBaseCompleted;
    QPushButton *btn_LoadReachabilityFile;
    QPushButton *btn_showUnionMap;
    QPushButton *btn_ClearUnionMap;
    QComboBox *combo_opGroup;
    QLabel *label_7;
    QCheckBox *show_umodel_checkBox;

    void setupUi(QWidget *BasePlacementWidget)
    {
        if (BasePlacementWidget->objectName().isEmpty())
            BasePlacementWidget->setObjectName(QString::fromUtf8("BasePlacementWidget"));
        BasePlacementWidget->resize(809, 644);
        gridLayout = new QGridLayout(BasePlacementWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        progressBar = new QProgressBar(BasePlacementWidget);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setValue(24);

        gridLayout->addWidget(progressBar, 1, 1, 1, 1);

        tabWidget = new QTabWidget(BasePlacementWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_1 = new QWidget();
        tab_1->setObjectName(QString::fromUtf8("tab_1"));
        gridLayout_2 = new QGridLayout(tab_1);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        label = new QLabel(tab_1);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        btn_ClearAllPoints = new QPushButton(tab_1);
        btn_ClearAllPoints->setObjectName(QString::fromUtf8("btn_ClearAllPoints"));

        gridLayout_2->addWidget(btn_ClearAllPoints, 10, 3, 1, 1);

        btn_SavePath = new QPushButton(tab_1);
        btn_SavePath->setObjectName(QString::fromUtf8("btn_SavePath"));

        gridLayout_2->addWidget(btn_SavePath, 10, 1, 1, 1);

        btn_LoadPath = new QPushButton(tab_1);
        btn_LoadPath->setObjectName(QString::fromUtf8("btn_LoadPath"));

        gridLayout_2->addWidget(btn_LoadPath, 10, 0, 1, 1);

        treeView = new QTreeView(tab_1);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setRootIsDecorated(true);
        treeView->setUniformRowHeights(true);

        gridLayout_2->addWidget(treeView, 1, 0, 4, 4);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 9, 2, 1, 2);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        btnRemovePoint = new QPushButton(tab_1);
        btnRemovePoint->setObjectName(QString::fromUtf8("btnRemovePoint"));

        gridLayout_3->addWidget(btnRemovePoint, 6, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        gridLayout_3->addItem(verticalSpacer, 7, 0, 1, 1);

        groupBox_3 = new QGroupBox(tab_1);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        combo_robotModel = new QComboBox(groupBox_3);
        combo_robotModel->setObjectName(QString::fromUtf8("combo_robotModel"));
        combo_robotModel->setGeometry(QRect(0, 30, 111, 23));

        gridLayout_3->addWidget(groupBox_3, 2, 0, 2, 1);

        txtPointName = new QLineEdit(tab_1);
        txtPointName->setObjectName(QString::fromUtf8("txtPointName"));

        gridLayout_3->addWidget(txtPointName, 5, 0, 1, 1);

        label_3 = new QLabel(tab_1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_3->addWidget(label_3, 4, 0, 1, 1);


        gridLayout_2->addLayout(gridLayout_3, 7, 3, 1, 1);

        newPointLayout = new QGroupBox(tab_1);
        newPointLayout->setObjectName(QString::fromUtf8("newPointLayout"));
        newPointLayout->setAutoFillBackground(false);
        newPointLayout->setStyleSheet(QString::fromUtf8("background-color: rgb(226, 226, 226)"));
        FormLayoutObjectPose_2 = new QFormLayout(newPointLayout);
        FormLayoutObjectPose_2->setObjectName(QString::fromUtf8("FormLayoutObjectPose_2"));
        FormLayoutObjectPose_2->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        groupBox = new QGroupBox(newPointLayout);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        groupBox->setFont(font);

        FormLayoutObjectPose_2->setWidget(0, QFormLayout::LabelRole, groupBox);

        label_2 = new QLabel(newPointLayout);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        FormLayoutObjectPose_2->setWidget(2, QFormLayout::LabelRole, label_2);

        xLabel = new QLabel(newPointLayout);
        xLabel->setObjectName(QString::fromUtf8("xLabel"));

        FormLayoutObjectPose_2->setWidget(5, QFormLayout::LabelRole, xLabel);

        LineEditX = new QLineEdit(newPointLayout);
        LineEditX->setObjectName(QString::fromUtf8("LineEditX"));
        LineEditX->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(5, QFormLayout::FieldRole, LineEditX);

        yLabel = new QLabel(newPointLayout);
        yLabel->setObjectName(QString::fromUtf8("yLabel"));

        FormLayoutObjectPose_2->setWidget(7, QFormLayout::LabelRole, yLabel);

        LineEditY = new QLineEdit(newPointLayout);
        LineEditY->setObjectName(QString::fromUtf8("LineEditY"));
        LineEditY->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(7, QFormLayout::FieldRole, LineEditY);

        zLabel = new QLabel(newPointLayout);
        zLabel->setObjectName(QString::fromUtf8("zLabel"));

        FormLayoutObjectPose_2->setWidget(8, QFormLayout::LabelRole, zLabel);

        LineEditZ = new QLineEdit(newPointLayout);
        LineEditZ->setObjectName(QString::fromUtf8("LineEditZ"));
        LineEditZ->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(8, QFormLayout::FieldRole, LineEditZ);

        rxLabel = new QLabel(newPointLayout);
        rxLabel->setObjectName(QString::fromUtf8("rxLabel"));

        FormLayoutObjectPose_2->setWidget(9, QFormLayout::LabelRole, rxLabel);

        LineEditRx = new QLineEdit(newPointLayout);
        LineEditRx->setObjectName(QString::fromUtf8("LineEditRx"));
        LineEditRx->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(9, QFormLayout::FieldRole, LineEditRx);

        ryLabel = new QLabel(newPointLayout);
        ryLabel->setObjectName(QString::fromUtf8("ryLabel"));

        FormLayoutObjectPose_2->setWidget(10, QFormLayout::LabelRole, ryLabel);

        LineEditRy = new QLineEdit(newPointLayout);
        LineEditRy->setObjectName(QString::fromUtf8("LineEditRy"));
        LineEditRy->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(10, QFormLayout::FieldRole, LineEditRy);

        rzLabel = new QLabel(newPointLayout);
        rzLabel->setObjectName(QString::fromUtf8("rzLabel"));

        FormLayoutObjectPose_2->setWidget(12, QFormLayout::LabelRole, rzLabel);

        LineEditRz = new QLineEdit(newPointLayout);
        LineEditRz->setObjectName(QString::fromUtf8("LineEditRz"));
        LineEditRz->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));

        FormLayoutObjectPose_2->setWidget(12, QFormLayout::FieldRole, LineEditRz);

        btnAddPoint = new QPushButton(newPointLayout);
        btnAddPoint->setObjectName(QString::fromUtf8("btnAddPoint"));

        FormLayoutObjectPose_2->setWidget(13, QFormLayout::LabelRole, btnAddPoint);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        FormLayoutObjectPose_2->setItem(1, QFormLayout::SpanningRole, verticalSpacer_4);


        gridLayout_2->addWidget(newPointLayout, 7, 0, 1, 3);

        tabWidget->addTab(tab_1, QString());
        label->raise();
        treeView->raise();
        btn_LoadPath->raise();
        btn_SavePath->raise();
        newPointLayout->raise();
        btn_ClearAllPoints->raise();
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        targetPoint = new QPushButton(tab_2);
        targetPoint->setObjectName(QString::fromUtf8("targetPoint"));
        targetPoint->setGeometry(QRect(10, 360, 181, 51));
        groupBox_2 = new QGroupBox(tab_2);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 90, 381, 191));
        QFont font1;
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setItalic(false);
        font1.setWeight(75);
        groupBox_2->setFont(font1);
        groupBox_2->setStyleSheet(QString::fromUtf8("background-color: rgb(226, 226, 226)"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 40, 161, 31));
        lnEdit_BaseLocSize = new QLineEdit(groupBox_2);
        lnEdit_BaseLocSize->setObjectName(QString::fromUtf8("lnEdit_BaseLocSize"));
        lnEdit_BaseLocSize->setGeometry(QRect(240, 40, 113, 27));
        lnEdit_BaseLocSize->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));
        combo_planGroup = new QComboBox(groupBox_2);
        combo_planGroup->setObjectName(QString::fromUtf8("combo_planGroup"));
        combo_planGroup->setGeometry(QRect(110, 130, 181, 23));
        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(40, 130, 51, 31));
        lnEdit_SpSize = new QLineEdit(groupBox_2);
        lnEdit_SpSize->setObjectName(QString::fromUtf8("lnEdit_SpSize"));
        lnEdit_SpSize->setGeometry(QRect(240, 80, 113, 27));
        lnEdit_SpSize->setStyleSheet(QString::fromUtf8("border-style: outset;\n"
"border-width: 2px;\n"
"border-color :rgb(48, 48, 48)"));
        label_8 = new QLabel(groupBox_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 70, 191, 31));
        lbl_placeBaseCompleted = new QLabel(tab_2);
        lbl_placeBaseCompleted->setObjectName(QString::fromUtf8("lbl_placeBaseCompleted"));
        lbl_placeBaseCompleted->setGeometry(QRect(10, 290, 461, 41));
        QFont font2;
        font2.setPointSize(12);
        font2.setBold(true);
        font2.setItalic(true);
        font2.setWeight(75);
        lbl_placeBaseCompleted->setFont(font2);
        btn_LoadReachabilityFile = new QPushButton(tab_2);
        btn_LoadReachabilityFile->setObjectName(QString::fromUtf8("btn_LoadReachabilityFile"));
        btn_LoadReachabilityFile->setGeometry(QRect(10, 10, 371, 31));
        btn_showUnionMap = new QPushButton(tab_2);
        btn_showUnionMap->setObjectName(QString::fromUtf8("btn_showUnionMap"));
        btn_showUnionMap->setGeometry(QRect(10, 50, 181, 27));
        btn_ClearUnionMap = new QPushButton(tab_2);
        btn_ClearUnionMap->setObjectName(QString::fromUtf8("btn_ClearUnionMap"));
        btn_ClearUnionMap->setGeometry(QRect(200, 50, 181, 27));
        combo_opGroup = new QComboBox(tab_2);
        combo_opGroup->setObjectName(QString::fromUtf8("combo_opGroup"));
        combo_opGroup->setGeometry(QRect(200, 360, 131, 23));
        label_7 = new QLabel(tab_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(220, 320, 71, 31));
        show_umodel_checkBox = new QCheckBox(tab_2);
        show_umodel_checkBox->setObjectName(QString::fromUtf8("show_umodel_checkBox"));
        show_umodel_checkBox->setEnabled(true);
        show_umodel_checkBox->setGeometry(QRect(200, 390, 181, 21));
        show_umodel_checkBox->setChecked(false);
        tabWidget->addTab(tab_2, QString());

        gridLayout->addWidget(tabWidget, 0, 1, 1, 1);


        retranslateUi(BasePlacementWidget);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(BasePlacementWidget);
    } // setupUi

    void retranslateUi(QWidget *BasePlacementWidget)
    {
        BasePlacementWidget->setWindowTitle(QApplication::translate("BasePlacementWidget", "Form", nullptr));
        label->setText(QApplication::translate("BasePlacementWidget", "Task Poses", nullptr));
        btn_ClearAllPoints->setText(QApplication::translate("BasePlacementWidget", "Clear All Poses", nullptr));
        btn_SavePath->setText(QApplication::translate("BasePlacementWidget", "Save Task", nullptr));
        btn_LoadPath->setText(QApplication::translate("BasePlacementWidget", "Load Task", nullptr));
        btnRemovePoint->setText(QApplication::translate("BasePlacementWidget", "Remove Pose", nullptr));
        groupBox_3->setTitle(QApplication::translate("BasePlacementWidget", "RobotModel", nullptr));
        label_3->setText(QApplication::translate("BasePlacementWidget", "Point", nullptr));
        groupBox->setTitle(QApplication::translate("BasePlacementWidget", "Add New Task Poses", nullptr));
        label_2->setText(QApplication::translate("BasePlacementWidget", "Set Position/Orientation", nullptr));
        xLabel->setText(QApplication::translate("BasePlacementWidget", "X (m)", nullptr));
        yLabel->setText(QApplication::translate("BasePlacementWidget", "Y (m)", nullptr));
        zLabel->setText(QApplication::translate("BasePlacementWidget", "Z (m)", nullptr));
        rxLabel->setText(QApplication::translate("BasePlacementWidget", "Rx (deg)", nullptr));
        ryLabel->setText(QApplication::translate("BasePlacementWidget", "Ry (deg)", nullptr));
        rzLabel->setText(QApplication::translate("BasePlacementWidget", "Rz (deg)", nullptr));
        btnAddPoint->setText(QApplication::translate("BasePlacementWidget", "Add Point", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_1), QApplication::translate("BasePlacementWidget", "Make Task", nullptr));
        targetPoint->setText(QApplication::translate("BasePlacementWidget", "Find Base", nullptr));
        groupBox_2->setTitle(QApplication::translate("BasePlacementWidget", "Base Placement Paramentes", nullptr));
        label_5->setText(QApplication::translate("BasePlacementWidget", "Number of Base Location", nullptr));
        label_6->setText(QApplication::translate("BasePlacementWidget", "Method", nullptr));
        label_8->setText(QApplication::translate("BasePlacementWidget", "Number of High Score Sphere", nullptr));
        lbl_placeBaseCompleted->setText(QString());
        btn_LoadReachabilityFile->setText(QApplication::translate("BasePlacementWidget", "Load Inverse Reachability File", nullptr));
        btn_showUnionMap->setText(QApplication::translate("BasePlacementWidget", "Show Union Map", nullptr));
        btn_ClearUnionMap->setText(QApplication::translate("BasePlacementWidget", "Clear Map", nullptr));
        label_7->setText(QApplication::translate("BasePlacementWidget", "Ouput Type", nullptr));
        show_umodel_checkBox->setText(QApplication::translate("BasePlacementWidget", "Show unreachable models", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("BasePlacementWidget", "Find Base", nullptr));
    } // retranslateUi

};

namespace Ui {
    class BasePlacementWidget: public Ui_BasePlacementWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_PLACEMENT_WIDGET_H
