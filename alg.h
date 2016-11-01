#pragma once

#include "mainwindow.h"

class MainWindow;
#include "datagram.h"

struct KalmanData
{
	double Kk, PMinusK, Pk, PkMinus1, R, Zk, Xk, XMinusK, XkMinus1;
	double dataSumm;//сумма для хранения 10 показаний приемника, чтобы рассчитать R
};

/*
 * This class is intended for math processing of the input messages from 8 anchors as series of time delays in picoseconds
*/
class Alg: public QObject
{
    Q_OBJECT
public:
	Alg();
    Alg(MainWindow* wnd = 0);
    virtual ~Alg();

    void init();

    bool Pair_Analyzing(const POINT3D* pt1,const POINT3D* pt2, POINT3D* ptRet);
    virtual bool ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint) = 0;
    virtual void process_nav(const ANC_MSG* datagram, POINT3D* retPoint) = 0;
	
	virtual int processMarkFilter(int tag_number) = 0;
    //фильтр первого порядка (RC), фильтр отметок
	virtual double mark_filter(int tag, int anc, double d) = 0;

	//virtual int processKalmanFilter(int tag_number) = 0;
	//фильтр Калмана
	//virtual double KalmanFilter(int tag, int anc, double d) = 0;


    virtual int prepare_data(int tag) = 0;
    double find_max_m(void);
    void anc_dist(void);
	void WriteToFile(const ANC_MSG* datagram, POINT3D* retPoint, FILE* file);
	void WriteRawDataToFile(const ANC_MSG* datagram, int anchor_number, double data, FILE* file);
	inline void setDatagramKalmanCount(int val) { datagramKalmanCount = val; }
	inline void resetKalmanFilterVariables()
	{
		setDatagramKalmanCount(0);
		for (int i = 0; i < ANCHORS_NUMBER; i++)
		{
			kalmanData[i].R = 0;
			kalmanData[i].PMinusK = 1;
			kalmanData[i].Pk = 1;
			kalmanData[i].PkMinus1 = 1;
			kalmanData[i].XMinusK = 0;
			kalmanData[i].XkMinus1 = 0;
			kalmanData[i].dataSumm = 0;
		}
	}
protected:
    int64 t_marks[TAGS_NUMBER][ANCHORS_NUMBER];		//time delay marks array 15x8 (picoseconds)
	int64 t_marks_tag0_K_estimate[37][8];		        //time delay marks array 1x8 (picoseconds) for 0 tag estimate for K[]
														//размерность 37х8 потому что тут записываются только 8 значений от приемников для ОДНОГО датчика за 37 циклов !
    int64  m_marks[ANCHORS_NUMBER];                   // prepared deltas for one tag
    POINT3D anchor[ANCHORS_NUMBER];						// anchor positions (m)
    POINT3D tag[TAGS_NUMBER];							// calculated tags coordinates

	int arrAnchVal[ANCHORS_NUMBER];//массив РАЗНОСТЕЙ задержек времен прихода сигналов / массив РАЗНОСТЕЙ расстояний приемников сигналов
    int a[4];//массив для хранения текущей комбинации 4 из 8 (номера четырех текущих приемников сигнала)
             //(4-х элементное подмножество из множества чисел {1...8}) 
    int sync_series;// series control
    int adj;// adjustment mode to set anchors antennas delays
    double anc0dist[ANCHORS_NUMBER];//distance anchor[i] to anchor[0]
	double ant_delay[ANCHORS_NUMBER];
	QString file_line_string;
	QString file_line_string_rawdata;

    //координаты приемников
    float u[ANCHORS_NUMBER];
	float v[ANCHORS_NUMBER];
	float w[ANCHORS_NUMBER];

    POINT3D* pt1;
    POINT3D* pt2;
    POINT3D* ptRet;
    POINT3D* p3d;
	POINT3D* ptRawData;
    MainWindow* mainWindow;	
	KalmanData kalmanData[ANCHORS_NUMBER];
	int datagramKalmanCount;
	float K[37][8];//массив для хранения коэффициентов К
	float Kav[8];//массив для хранения средних коэффициентов К для нулевого датчика
	int marks_number;//номер (или количество) серии данных
};


