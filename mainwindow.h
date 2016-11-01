/****************************************************************************
** VRST, Virtual reality space tracking
** Copyright (C) Digital Gravitation 2016
** Contact:

**
****************************************************************************/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdlib.h>
#include <QMainWindow>
#include <QVBoxLayout>
#include "ntw.h"
//#include "alg.h"
//#include "alg1.h"
//#include "Alg2.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QAction;
QT_END_NAMESPACE

class Ntw;
//class Alg;
class Alg1;
class Alg2;

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget/*QMainWindow*/
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    inline QLabel *  getStatusLabel(){return statusLabel;}
    inline QLabel *  getStatusLabel2(){return statusLabel2;}
    void SetOutput(QString txt,QString txt2,QString txt3,QString txt4,QString txt5);
	void SetOutput2(QString txt, QString txt2, QString txt3);
	void SetOutput3(QString txt1, QString txt2, QString txt3, QString txt4, QString txt5, QString txt6, QString txt7, QString txt8,
		QString txt9, QString txt10, QString txt11, QString txt12, QString txt13, QString txt14, QString txt15, QString txt16,
		QString txt17, QString txt18, QString txt19, QString txt20, QString txt21, QString txt22, QString txt23, QString txt24);
    void SetLabelText(QString txt);
    void SetLabelText2(QString txt);
	
	void SetMMarksWOCorrection1(QString txt);
	void SetMMarksWOCorrection2(QString txt);
	void SetMMarksWOCorrection3(QString txt);
	void SetMMarksWOCorrection4(QString txt);
	void SetMMarksWOCorrection5(QString txt);
	void SetMMarksWOCorrection6(QString txt);
	void SetMMarksWOCorrection7(QString txt);
	void SetMMarksWOCorrection8(QString txt);	

	void SetMMarks1(QString txt);
	void SetMMarks2(QString txt);
	void SetMMarks3(QString txt);
	void SetMMarks4(QString txt);
	void SetMMarks5(QString txt);
	void SetMMarks6(QString txt);
	void SetMMarks7(QString txt);
	void SetMMarks8(QString txt);
	void SetKav1(QString txt);
	void SetKav2(QString txt);
	void SetKav3(QString txt);
	void SetKav4(QString txt);
	void SetKav5(QString txt);
	void SetKav6(QString txt);
	void SetKav7(QString txt);
	void SetKav8(QString txt);
    void SetLabelTextAnchorXLabel(QString txt);
    void SetLabelTextAnchorYLabel(QString txt);
    void SetLabelTextAnchorZLabel(QString txt);
	Alg1* getAlg1();
	Alg2* getAlg2();
	inline FILE* getCoordFile1() { return file_out; }
	inline FILE* getCoordFile2() { return file_out2; }
	inline FILE* getRawDataFile() { return file_raw_data; }

private:
    Ui::MainWindow *ui;
    QWidget *widget;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    QHBoxLayout *statuslabelLayout;
	QHBoxLayout *labelLayout2;
    QHBoxLayout *AnchorlabelLayout;
	QHBoxLayout *MMarksLayout1;
	QHBoxLayout *MMarksLayout2;
	QHBoxLayout *MMarksLayout3;
	QHBoxLayout *MMarksLayout4;
	QHBoxLayout *MMarksLayout5;
	QHBoxLayout *MMarksLayout6;
	QHBoxLayout *MMarksLayout7;
	QHBoxLayout *MMarksLayout8;


    QLabel *statusLabel;
    QLabel *statusLabel2;
	QLabel *alg1Label;
	QLabel *alg2Label;
    QLabel *AnchorXLabel;
    QLabel *AnchorYLabel;
    QLabel *AnchorZLabel;
	QLabel *AnchorX2Label;
	QLabel *AnchorY2Label;
	QLabel *AnchorZ2Label;
    QPushButton *startButton;
    QPushButton *stopButton;

	//m_marks[i] without correction
	QLabel *LabelMMarksWOCorrection1;
	QLabel *LabelMMarksWOCorrection2;
	QLabel *LabelMMarksWOCorrection3;
	QLabel *LabelMMarksWOCorrection4;
	QLabel *LabelMMarksWOCorrection5;
	QLabel *LabelMMarksWOCorrection6;
	QLabel *LabelMMarksWOCorrection7;
	QLabel *LabelMMarksWOCorrection8;
	
	//m_marks[i] with correction
	QLabel *LabelMMarks1;
	QLabel *LabelMMarks2;
	QLabel *LabelMMarks3;
	QLabel *LabelMMarks4;
	QLabel *LabelMMarks5;
	QLabel *LabelMMarks6;
	QLabel *LabelMMarks7;
	QLabel *LabelMMarks8;

	QLabel *LabelKav1;
	QLabel *LabelKav2;
	QLabel *LabelKav3;
	QLabel *LabelKav4;
	QLabel *LabelKav5;
	QLabel *LabelKav6;
	QLabel *LabelKav7;
	QLabel *LabelKav8;

    Ntw* ntw;//network class
    Alg1* alg1;//algorithm class
	Alg2* alg2;

    bool state;
	FILE *file_out;
	FILE *file_out2;
	FILE *file_raw_data;

private slots:
    void stop();
    void start();
};

#endif // MAINWINDOW_H
