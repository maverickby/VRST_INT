#include "mainwindow.h"
#include "alg1.h"
#include "Alg2.h"
#include "ui_mainwindow.h"

#include <QtWidgets>

MainWindow::MainWindow(QWidget *parent) :
	QWidget(parent), /*QMainWindow(parent),*/
    ui(new Ui::MainWindow)
{
    //ui->setupUi((QMainWindow*)this);
    statuslabelLayout = new QHBoxLayout();
    statusLabel = new QLabel(tr("Listening for TDOA UDP server messages"));
    statusLabel->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel);

    statusLabel2 = new QLabel(tr(""));
    statusLabel2->setWordWrap(true);
    statuslabelLayout->addWidget(statusLabel2);

	QFrame* line = new QFrame();
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);	
	QFrame* line2 = new QFrame();
	line2->setFrameShape(QFrame::HLine);
	line2->setFrameShadow(QFrame::Sunken);

	labelLayout2 = new QHBoxLayout();
	alg2Label = new QLabel(tr("Algoritm 2"));
	alg2Label->setWordWrap(true);	
	labelLayout2->addWidget(alg2Label);
	
	LabelMMarksWOCorrection1 = new QLabel(tr(""));
	LabelMMarksWOCorrection2 = new QLabel(tr(""));
	LabelMMarksWOCorrection3 = new QLabel(tr(""));
	LabelMMarksWOCorrection4 = new QLabel(tr(""));
	LabelMMarksWOCorrection5 = new QLabel(tr(""));
	LabelMMarksWOCorrection6 = new QLabel(tr(""));
	LabelMMarksWOCorrection7 = new QLabel(tr(""));
	LabelMMarksWOCorrection8 = new QLabel(tr(""));

	LabelMMarks1 = new QLabel(tr(""));
	LabelMMarks2 = new QLabel(tr(""));
	LabelMMarks3 = new QLabel(tr(""));
	LabelMMarks4 = new QLabel(tr(""));
	LabelMMarks5 = new QLabel(tr(""));
	LabelMMarks6 = new QLabel(tr(""));
	LabelMMarks7 = new QLabel(tr(""));
	LabelMMarks8 = new QLabel(tr(""));

	LabelKav1 = new QLabel(tr(""));;
	LabelKav2 = new QLabel(tr(""));;
	LabelKav3 = new QLabel(tr(""));;
	LabelKav4 = new QLabel(tr(""));;
	LabelKav5 = new QLabel(tr(""));;
	LabelKav6 = new QLabel(tr(""));;
	LabelKav7 = new QLabel(tr(""));;
	LabelKav8 = new QLabel(tr(""));;
	
	//m_marks without correction[i] + Kav[i] + m_marks[i]
	MMarksLayout1 = new QHBoxLayout();
	MMarksLayout1->addWidget(LabelMMarksWOCorrection1);
	MMarksLayout1->addWidget(LabelKav1);
	MMarksLayout1->addWidget(LabelMMarks1);	
	MMarksLayout2 = new QHBoxLayout();
	MMarksLayout2->addWidget(LabelMMarksWOCorrection2);
	MMarksLayout2->addWidget(LabelKav2);
	MMarksLayout2->addWidget(LabelMMarks2);
	MMarksLayout3 = new QHBoxLayout();
	MMarksLayout3->addWidget(LabelMMarksWOCorrection3);
	MMarksLayout3->addWidget(LabelKav3);
	MMarksLayout3->addWidget(LabelMMarks3);
	MMarksLayout4 = new QHBoxLayout();
	MMarksLayout4->addWidget(LabelMMarksWOCorrection4);
	MMarksLayout4->addWidget(LabelKav4);
	MMarksLayout4->addWidget(LabelMMarks4);
	MMarksLayout5 = new QHBoxLayout();
	MMarksLayout5->addWidget(LabelMMarksWOCorrection5);
	MMarksLayout5->addWidget(LabelKav5);
	MMarksLayout5->addWidget(LabelMMarks5);
	MMarksLayout6 = new QHBoxLayout();
	MMarksLayout6->addWidget(LabelMMarksWOCorrection6);
	MMarksLayout6->addWidget(LabelKav6);
	MMarksLayout6->addWidget(LabelMMarks6);
	MMarksLayout7 = new QHBoxLayout();
	MMarksLayout7->addWidget(LabelMMarksWOCorrection7);
	MMarksLayout7->addWidget(LabelKav7);
	MMarksLayout7->addWidget(LabelMMarks7);
	MMarksLayout8 = new QHBoxLayout();
	MMarksLayout8->addWidget(LabelMMarksWOCorrection8);
	MMarksLayout8->addWidget(LabelKav8);
	MMarksLayout8->addWidget(LabelMMarks8);
	
	QLabel* algLabel = new QLabel(tr("Algoritm 1"));
	algLabel->setWordWrap(true);

    AnchorlabelLayout = new QHBoxLayout();
    AnchorXLabel = new QLabel(tr("Anchor X:    "));
    AnchorYLabel = new QLabel(tr("Anchor Y:    "));
    AnchorZLabel = new QLabel(tr("Anchor Z:    "));
    AnchorlabelLayout->addWidget(AnchorXLabel);
    AnchorlabelLayout->addWidget(AnchorYLabel);
    AnchorlabelLayout->addWidget(AnchorZLabel);

	QHBoxLayout* AnchorlabelLayout2 = new QHBoxLayout();
	
	AnchorX2Label = new QLabel(tr("Anchor X:    "));
	AnchorY2Label = new QLabel(tr("Anchor Y:    "));
	AnchorZ2Label = new QLabel(tr("Anchor Z:    "));
	
	AnchorlabelLayout2->addWidget(AnchorX2Label);
	AnchorlabelLayout2->addWidget(AnchorY2Label);
	AnchorlabelLayout2->addWidget(AnchorZ2Label);

    stopButton = new QPushButton(tr("&Stop"));
    startButton = new QPushButton(tr("&Start"));

    buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(stopButton);    

    mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(statuslabelLayout);

	mainLayout->addLayout(MMarksLayout1);
	mainLayout->addLayout(MMarksLayout2);
	mainLayout->addLayout(MMarksLayout3);
	mainLayout->addLayout(MMarksLayout4);
	mainLayout->addLayout(MMarksLayout5);
	mainLayout->addLayout(MMarksLayout6);
	mainLayout->addLayout(MMarksLayout7);
	mainLayout->addLayout(MMarksLayout8);

	mainLayout->addWidget(line);
	mainLayout->addWidget(algLabel);
    mainLayout->addLayout(AnchorlabelLayout);
	mainLayout->addWidget(line2);
	mainLayout->addLayout(labelLayout2);
	mainLayout->addLayout(AnchorlabelLayout2);
	//mainLayout->addSpacing(40);
    mainLayout->addLayout(buttonLayout);
	

    mainLayout->setAlignment(buttonLayout,Qt::AlignRight);

    setLayout(mainLayout);

    stopButton->setDisabled(true);

    //Технология пространственного трекинга для систем виртуальной реальности (VR)
    setWindowTitle(tr("VR space tracking"));//VR space tracking

    resize(480,400);
    setFixedSize(this->size());

    //start network
    ntw = new Ntw(this);
    //create algorithm instances
    alg1 = new Alg1(this);
    alg2 = new Alg2(this);
	file_out = fopen("coordinates_out.txt", "wt");
	file_out2 = fopen("coordinates_out2.txt", "wt");
	file_raw_data = fopen("raw_data.txt", "wt");	

    connect(stopButton, SIGNAL(clicked()), this, SLOT(stop()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(start()));
}

MainWindow::~MainWindow()
{
	delete ui;
    /*delete statusLabel;
    delete statusLabel2;
    delete AnchorXLabel;
    delete AnchorYLabel;
    delete AnchorZLabel;
    delete stopButton;
    delete startButton;

    delete statuslabelLayout;
    delete AnchorlabelLayout;
    delete buttonLayout;
    delete mainLayout;*/
    
    //delete ntw;
    //delete alg;
/*	if(file_out)
		fclose(file_out);
	if (file_out2)
		fclose(file_out2);*/
}

Alg1* MainWindow::getAlg1()
{
	return alg1;
}

Alg2* MainWindow::getAlg2()
{
	return alg2;
}

void MainWindow::start()
{
    ntw->start();
    startButton->setDisabled(true);
    stopButton->setDisabled(false);
    //alg->DirectCalculationMethod();
	file_out = fopen("coordinates_out.txt", "wt");
	file_out2 = fopen("coordinates_out2.txt", "wt");
	file_raw_data = fopen("raw_data.txt", "wt");
	
	qDebug("start");
}

void MainWindow::stop()
{
    ntw->stop();
    startButton->setDisabled(false);
    stopButton->setDisabled(true);
	if (file_out)
		fclose(file_out);
	if (file_out2)
		fclose(file_out2);
	if (file_raw_data)
		fclose(file_raw_data);
	qDebug("stop");
}

void MainWindow::SetOutput(QString txt,QString txt2,QString txt3,QString txt4,QString txt5)
{
    SetLabelText(txt);
    SetLabelText2(txt2);
    SetLabelTextAnchorXLabel(txt3);
    SetLabelTextAnchorYLabel(txt4);
    SetLabelTextAnchorZLabel(txt5);
}

void MainWindow::SetOutput3(QString txt1, QString txt2, QString txt3, QString txt4, QString txt5, QString txt6, QString txt7, QString txt8,
	QString txt9, QString txt10, QString txt11, QString txt12, QString txt13, QString txt14, QString txt15, QString txt16,
	QString txt17, QString txt18, QString txt19, QString txt20, QString txt21, QString txt22, QString txt23, QString txt24)
{
	SetMMarksWOCorrection1(txt1);
	SetMMarksWOCorrection2(txt2);
	SetMMarksWOCorrection3(txt3);
	SetMMarksWOCorrection4(txt4);
	SetMMarksWOCorrection5(txt5);
	SetMMarksWOCorrection6(txt6);
	SetMMarksWOCorrection7(txt7);
	SetMMarksWOCorrection8(txt8);

	SetMMarks1(txt9);
	SetMMarks2(txt10);
	SetMMarks3(txt11);
	SetMMarks4(txt12);
	SetMMarks5(txt13);
	SetMMarks6(txt14);
	SetMMarks7(txt15);
	SetMMarks8(txt16);
	SetKav1(txt17);
	SetKav2(txt18);
	SetKav3(txt19);
	SetKav4(txt20);
	SetKav5(txt21);
	SetKav6(txt22);
	SetKav7(txt23);
	SetKav8(txt24);

}

void MainWindow::SetMMarksWOCorrection1(QString txt)
{
	LabelMMarksWOCorrection1->setText(txt);
}

void MainWindow::SetMMarksWOCorrection2(QString txt)
{
	LabelMMarksWOCorrection2->setText(txt);
}

void MainWindow::SetMMarksWOCorrection3(QString txt)
{
	LabelMMarksWOCorrection3->setText(txt);
}

void MainWindow::SetMMarksWOCorrection4(QString txt)
{
	LabelMMarksWOCorrection4->setText(txt);
}

void MainWindow::SetMMarksWOCorrection5(QString txt)
{
	LabelMMarksWOCorrection5->setText(txt);
}

void MainWindow::SetMMarksWOCorrection6(QString txt)
{
	LabelMMarksWOCorrection6->setText(txt);
}

void MainWindow::SetMMarksWOCorrection7(QString txt)
{
	LabelMMarksWOCorrection7->setText(txt);
}

void MainWindow::SetMMarksWOCorrection8(QString txt)
{
	LabelMMarksWOCorrection8->setText(txt);
}

//////////////////

void MainWindow::SetMMarks1(QString txt)
{
	LabelMMarks1->setText(txt);
}
void MainWindow::SetMMarks2(QString txt)
{
	LabelMMarks2->setText(txt);
}
void MainWindow::SetMMarks3(QString txt)
{
	LabelMMarks3->setText(txt);
}
void MainWindow::SetMMarks4(QString txt)
{
	LabelMMarks4->setText(txt);
}
void MainWindow::SetMMarks5(QString txt)
{
	LabelMMarks5->setText(txt);
}
void MainWindow::SetMMarks6(QString txt)
{
	LabelMMarks6->setText(txt);
}
void MainWindow::SetMMarks7(QString txt)
{
	LabelMMarks7->setText(txt);
}
void MainWindow::SetMMarks8(QString txt)
{
	LabelMMarks8->setText(txt);
}
///////////
void MainWindow::SetKav1(QString txt)
{
	LabelKav1->setText(txt);
}
void MainWindow::SetKav2(QString txt)
{
	LabelKav2->setText(txt);
}
void MainWindow::SetKav3(QString txt)
{
	LabelKav3->setText(txt);
}
void MainWindow::SetKav4(QString txt)
{
	LabelKav4->setText(txt);
}
void MainWindow::SetKav5(QString txt)
{
	LabelKav5->setText(txt);
}
void MainWindow::SetKav6(QString txt)
{
	LabelKav6->setText(txt);
}
void MainWindow::SetKav7(QString txt)
{
	LabelKav7->setText(txt);
}
void MainWindow::SetKav8(QString txt)
{
	LabelKav8->setText(txt);
}

void MainWindow::SetOutput2(QString txt, QString txt2, QString txt3)
{
	AnchorX2Label->setText(txt);
	AnchorY2Label->setText(txt2);
	AnchorZ2Label->setText(txt3);	
}

void MainWindow::SetLabelText(QString txt)
{
    statusLabel->setText(txt);
}

void MainWindow::SetLabelText2(QString txt)
{
    statusLabel2->setText(txt);
}

void MainWindow::SetLabelTextAnchorXLabel(QString txt)
{
    AnchorXLabel->setText(txt);
}
void MainWindow::SetLabelTextAnchorYLabel(QString txt)
{
    AnchorYLabel->setText(txt);
}
void MainWindow::SetLabelTextAnchorZLabel(QString txt)
{
    AnchorZLabel->setText(txt);
}



