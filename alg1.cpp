#include "alg1.h"
#include <cmath>
#include <QDebug>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#define BUFFER_SIZE 128



Alg1::Alg1(MainWindow* wnd):Alg(wnd)
{
	//mainWindow = wnd;
	//init();
	init_();
}

void Alg1::init_()
{
	for (int i = 0; i<3; i++)
	{
		arrJ[i] = 0;
	}
	
	memset(a_, 0, sizeof(a_));//clear trash in the array
	memset(arrAnchVal, 0, sizeof(arrAnchVal));//clear trash in the array
	flagOTagProcessing = true;
}

Alg1::~Alg1()
{

}

int Alg1::processMarkFilter(int tag_number)
{
	int i;
	int j = 0;

	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		//if (fabs(m_marks[i]) < XY_DIMENSION)   // if delta > XY_DIMENSION  -  ignore it
		{
			m_marks[i] = mark_filter(tag_number, i, m_marks[i]);

			j++;
		}
	}
	return(j);
}

// marks filter
//фильтр первого порядка (RC), фильтр отметок
double Alg1::mark_filter(int tag, int anc, double d)
{
	static double f[TAGS_NUMBER][ANCHORS_NUMBER];
	double k = 20.0;
	f[tag][anc] = f[tag][anc] + fabs(d) - (f[tag][anc] / k);
	return(f[tag][anc] / k);
}

//some magic here is
int Alg1::processKalmanFilter(int tag_number)
{
	int i;
	int j = 0;

	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		//if (fabs(m_marks[i]) < XY_DIMENSION)   // if delta > XY_DIMENSION  -  ignore it
		{
			if (datagramKalmanCount > 9)
			{
				if (datagramKalmanCount == 10)
					kalmanData[i].R = kalmanData[i].dataSumm / 100;
				m_marks[i] = KalmanFilter(tag_number, i, m_marks[i]);
				//datagramKalmanCount = 0;//reset datagram counter for Kalman filter's R estimate
				//kalmanData[i].dataSumm = 0;//reset data sum after Kalman filter processing
			}
			else
			{				
				kalmanData[i].dataSumm += m_marks[i];
			}

			j++;
		}
	}
	return(j);
}

//фильтр Калмана
double Alg1::KalmanFilter(int tag, int anc, double d)
{
	kalmanData[anc].XMinusK = kalmanData[anc].XkMinus1;
	kalmanData[anc].PMinusK = kalmanData[anc].PkMinus1;

	kalmanData[anc].Kk = kalmanData[anc].PMinusK / (kalmanData[anc].PMinusK + fabs(kalmanData[anc].R));
	kalmanData[anc].Xk = kalmanData[anc].XMinusK  + kalmanData[anc].Kk*(fabs(d) - fabs(kalmanData[anc].XMinusK));
	kalmanData[anc].Pk = (1 - fabs(kalmanData[anc].Kk)) * kalmanData[anc].PMinusK;
	
	kalmanData[anc].PkMinus1 = kalmanData[anc].Pk;
	kalmanData[anc].XkMinus1 = kalmanData[anc].Xk;
	
	if(d<0)
		return -1*kalmanData[anc].Xk;	
	else
		return kalmanData[anc].Xk;
}

int Alg1::prepare_data_0tag()
{
	int i;
	double a; /*d0*/
	int64 d0;
	d0 = 0;
	double dwt_time_units = 15.65;

	// fix time marks using distance to master	(work)
	for (int i = 0; i < 37;i++)
	{	
		for (int j = 0; j < ANCHORS_NUMBER; j++)
		{
			if (t_marks_tag0_K_estimate[i][j] != 0)
			{				
				if (adj == 0)     // work mode
					t_marks_tag0_K_estimate[i][j] += anc0dist[j];
			}
			else
				t_marks_tag0_K_estimate[i][j] = 0;  // no data
		}
	}	

	for (int i = 0; i < 37; i++)
	{
		d0 = t_marks_tag0_K_estimate[i][0];//получить значение 0го приемника
		for (int j = 0; j < ANCHORS_NUMBER; j++)
		{
			if (t_marks_tag0_K_estimate[i][j] != 0)
			{
				//change absolute marks  to delta (mark[i] - mark[0])
				t_marks_tag0_K_estimate[i][j] = t_marks_tag0_K_estimate[i][j] - d0;
				//t_marks_tag0_K_estimate[i][j] *= dwt_time_units;//умножать тут не надо, т.к. будет умножение в дальнейшем в process_K()
			}
			else
				t_marks_tag0_K_estimate[i][j] = 0;  // no data
		}
	}

	return(1);
}

bool Alg1::process_K()
{
	double Lir;
	double L0 = 2.12;
	double L4 = 3.13;
	double Ti;

	for (int i = 0; i < 37; i++)
	{
		for (int j = 0; j <= 3; j++)
		{
			Lir = L0 + t_marks_tag0_K_estimate[i][j] * 15.65 * 0.3 * 1e-3;
			K[i][j] = L0 / Lir;
		}

		for (int j = 4; j < 8; j++)
		{
			Ti = (t_marks_tag0_K_estimate[i][j] * L4 / L0) / (L4 / L0 - 1);
			Lir = Ti * 15.65 * 0.3 * 1e-3;
			K[i][j] = L4 / Lir;
		}
	}
	return true;
}

bool Alg1::process_Kav()
{	
	double sum=0;
	float KavI = 0;
	std::vector<float> Kaverage;
		
	for (int j = 0; j < 8; j++)
	{
		for (int i = 0; i < 37; i++)
			Kaverage.push_back(K[i][j]);
	
			std::sort(Kaverage.begin(), Kaverage.end());
			Kaverage.erase(Kaverage.begin(), Kaverage.begin() + 5);
			Kaverage.erase(Kaverage.end() - 4, Kaverage.end() );
			Kaverage.pop_back();

			for (int k = 0; k < Kaverage.size(); k++)
				sum += Kaverage[k];			
			
			KavI = sum / Kaverage.size();
			
			//анализ на +-10%
			if ( (KavI < (Kav[j] + Kav[j] * 0.1)) && (KavI > (Kav[j] - Kav[j] * 0.1)) || Kav[j]==0)
				Kav[j] = KavI;

			//Kav[j] = sum / Kaverage.size();
			Kaverage.clear();
			sum = 0;
	}
	return true;
}

bool Alg1::ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint)
{
	//TODO temp, move to another place !
	anc_dist();
	int i;
	int64 m0, m1, m2, m3, m4, m5, m6, m7;
	int64 m0_, m1_, m2_, m3_, m4_, m5_, m6_, m7_;
	float k1, k2, k3, k4, k5, k6, k7, k8;

	if (datagram->addr == 0)  // first packet in the series, process previous data
	{		
		if (flagOTagProcessing == false)
		{
			prepare_data_0tag();
			process_K();
			process_Kav();

			memset(t_marks_tag0_K_estimate, 0, sizeof(t_marks_tag0_K_estimate));//очистить массив t_marks_tag0_K_estimate[][]

			//show data
			m0_ = m_marks[0];//. * 15.65;
			m1_ = m_marks[1]; //* 15.65;
			m2_ = m_marks[2]; //* 15.65;
			m3_ = m_marks[3]; //* 15.65;
			m4_ = m_marks[4]; //* 15.65;
			m5_ = m_marks[5]; //* 15.65;
			m6_ = m_marks[6]; //* 15.65;
			m7_ = m_marks[7]; //* 15.65;
			
			m0 = m_marks[0] * Kav[0];
			m1 = m_marks[1] * Kav[1];
			m2 = m_marks[2] * Kav[2];
			m3 = m_marks[3] * Kav[3];
			m4 = m_marks[4] * Kav[4];
			m5 = m_marks[5] * Kav[5];
			m6 = m_marks[6] * Kav[6];
			m7 = m_marks[7] * Kav[7];

			mainWindow->SetOutput3(tr("m_marks0: %1").arg(m0_), tr("m_marks1: %1").arg(m1_), tr("m_marks2: %1").arg(m2_),
				tr("m_marks3: %1").arg(m3_), tr("m_marks4: %1").arg(m4_), tr("m_marks5: %1").arg(m5_),
				tr("m_marks6: %1").arg(m6_), tr("m_marks7: %1").arg(m7_),
				tr("Kav0: %1").arg(Kav[0]), tr("Kav1: %1").arg(Kav[1]), tr("Kav2: %1").arg(Kav[2]),
				tr("Kav3: %1").arg(Kav[3]), tr("Kav4: %1").arg(Kav[4]), tr("Kav5: %1").arg(Kav[5]),
				tr("Kav6: %1").arg(Kav[6]), tr("Kav7: %1").arg(Kav[7]),
				tr("m_marks0: %1").arg(m0), tr("m_marks1: %1").arg(m1), tr("m_marks2: %1").arg(m2),
				tr("m_marks3: %1").arg(m3), tr("m_marks4: %1").arg(m4), tr("m_marks5: %1").arg(m5),
				tr("m_marks6: %1").arg(m6), tr("m_marks7: %1").arg(m7));

			flagOTagProcessing = true;
			marks_number = -1;

			memset(m_marks, 0, sizeof(m_marks));//очистить массив m_marks[]
			
			//return true;
		}
		process_nav(datagram, retPoint); //prepare data and do navigation procedure
		memset(t_marks, 0, sizeof(t_marks));	   //clear marks
		//memset(t_marks_tag0_K_estimate, 0, sizeof(t_marks_tag0_K_estimate));//очистить массив t_marks_tag0_K_estimate[][]		
		sync_series = datagram->sync_n;		   //new #

		//если прошло 37 циклов, то устанавливаем flagOTagProcessing = false
		if (marks_number >= ONE_SEC_MARKS_NUMBER - 1)
			flagOTagProcessing = false;
		if (marks_number < ONE_SEC_MARKS_NUMBER-1)
			marks_number++;		

		datagramKalmanCount++;//add datagram counter for Kalman filter's R estimate		
	}

	// get new data for the anchor and process 1 second cycle for K estimate
	if (datagram->sync_n == sync_series)
	{		
		if (flagOTagProcessing == true)//copy marks to array t_marks_tag0_K_estimate[37][8] - 8 значений приемников для одного 0 датчика за 37 циклов
		{	
			memcpy((char *)&t_marks_tag0_K_estimate[marks_number][datagram->addr], datagram->time_mark[0], 5);//копируем данные ТОЛЬКО из 0 датчика !			
		}
		else
		{
			//copy marks - иначе считываем данные в обычном режиме
			for (i = 0; i < TAGS_NUMBER; i++)
				memcpy((char *)&t_marks[i][datagram->addr], datagram->time_mark[i], 5);

			int xx = 1;
		}		
	}

	//check for sensors data, NOT IMPLEMENTED YET
	if (datagram->length >(TAGS_NUMBER * 5))
	{/*
	 n = datagram->sd_tag;
	 for(i = 0; i < datagram->sd_tags; i++)
	 {
	 memcpy(&accel_gyro[n], &datagram->sens_data[i], 12);
	 disp_accel_gyro(n);
	 n ++;
	 if(n >= TAGS_NUMBER)
	 n = 0;
	 }*/
	}	
	return true;
}

void Alg1::process_nav(const ANC_MSG* datagram, POINT3D* retPoint)
{
	int i, count_anchors_ret;
	//float dwt_time_units = 15.65;
	for (i = 0; i < TAGS_NUMBER; i++)         // 0..14
	{
		if (prepare_data(i, datagram))
		{
			//make mark filter data processing
			//count_anchors_ret = processMarkFilter(i);

			//make Kalman filter data processing
			//count_anchors_ret = processKalmanFilter(i);

			//if (count_anchors_ret<4)//wrong situation !
			//	return;

			/*m_marks[0] = 963632963941;
			m_marks[1] = 963632963670;
			m_marks[2] = 963632963683;
			m_marks[3] = 963632963675;
			m_marks[4] = 963632963445;
			m_marks[5] = 963632963520;
			m_marks[6] = 963632963430;
			m_marks[7] = 963632963512;
			
			int64 d0;
			d0 = m_marks[0];

			for (int i = 0; i < 8;i++)
			{
				m_marks[i] = m_marks[i] - d0;
				m_marks[i] *= dwt_time_units;
			}*/	

			//call navigation algorithm here
			retPoint = DirectCalculationMethod(i);

			//show data
			mainWindow->SetOutput(tr("Anchor: %1").arg(datagram->addr), tr("Sync series_number: %1").arg(datagram->sync_n), tr("Anchor X: %1").arg(retPoint->x),
				tr("Anchor Y: %1").arg(retPoint->y), tr("Anchor Z: %1").arg(retPoint->z));

			//write results to the file
			WriteToFile(datagram, retPoint, mainWindow->getCoordFile1());
		}
	}
}

// distance anchor i to sync anchor 0 in DWT_TIME_UNITS
void Alg1::anc_dist(void)
{
	int i;
	for (i = 0; i < ANCHORS_NUMBER; i++)
		anc0dist[i] = sqrt(pow(anchor[i].x - anchor[0].x, 2) +
			pow(anchor[i].y - anchor[0].y, 2) +
			pow(anchor[i].z - anchor[0].z, 2)) / SPEED_OF_LIGHT / DWT_TIME_UNITS;
}

// ************  Time marks normalization functions

// to find maximum in m_marks array
double Alg1::find_max_m(void)
{
	double a = m_marks[0];
	int i;
	for (i = 1; i < ANCHORS_NUMBER; i++)
		if (m_marks[i] > a)
			a = m_marks[i];
	return(a);
}

int Alg1::prepare_data(int tag)
{
	return 0;
}

// copy t_mark for selected tag to m_mark
// check and fix overflow
// change time from DWT_TIME_UNITS to SECONDS
// change absolute mark value to delta (markN - mark0)
// return 0 if no data for master anchor (#0)
int Alg1::prepare_data(int tag, const ANC_MSG* datagram)
{
	int i;
	double a; /*d0*/
	int64 d0;
	d0 = 0;
	double dwt_time_units = 15.65;

	if (tag == 0)//0 датчик НЕ обрабатываем здесь, он обрабатывется в prepare_data_0tag()
		return 0;
	if (t_marks[tag][0] == 0)  //no data for master anchor
		return(0);
	// fix time marks using distance to master	(work)
	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		if (t_marks[tag][i] != 0)
		{
			m_marks[i] = (double)t_marks[tag][i];
			//if (adj == 0)     // work mode
			//	m_marks[i] += anc0dist[i];
		}
		else
			m_marks[i] = 0;  // no data
	}
	// check and fix 5 bytes numbers overflow
	a = find_max_m();
	for (i = 0; i < ANCHORS_NUMBER; i++)
	{
		//if ((m_marks[i]) && ((a - m_marks[i]) > HALF_T5))
			//m_marks[i] += MAX_T5;
		if (m_marks[i])	   // 0 = no data
		{
			// to seconds
			// array ant_delay  - meters
			//здесь разницы времен получения сигнала от маяка до i-го передатчика относительно
			//первого передатчика, в секундах (пикосекундах)
			//m_marks[i] = m_marks[i];// *DWT_TIME_UNITS + ant_delay[i] / SPEED_OF_LIGHT;
			m_marks[i] = m_marks[i] * dwt_time_units;
			if (i == 0)
				d0 = m_marks[0];
			//change absolute marks  to delta (mark[i] - mark[0])
			m_marks[i] = m_marks[i] - d0;
			//m_marks[i] *= dwt_time_units;

			//write results to the file
			WriteRawDataToFile(datagram, i, m_marks[i], mainWindow->getRawDataFile());
		}
		//else
		//    m_marks[i] = 10;   // set to 10m - will be removed
	}
	return(1);
}

POINT3D* Alg1::DirectCalculationMethod(int tag)
{
	//follow further variables declaration/use for optimization (CPU load)
	double x, y, z;
	int l, p, i;
	double t11, t21, t31, t41, t51, t61, t71, t81;
	int s, k;
	int index_anyJ;
	int u21, u31, u41, v21, v31, v41, w21, w31, w41;
	int tau12, tau13, tau14, tau32, tau42;
	float delta21, delta31, delta41;
	//long double alpha1, alpha2, beta1, beta2, gamma1, gamma2, g1, g2, c;
	float c;
	int alpha1, alpha2, beta1, beta2, gamma1, gamma2 /*g1, g2*/;
	int64 g1, g2;

	float A, B, C, D, E, F, G, H, I;
	float zl, xl, yl, xMinus, xPlus, yMinus, yPlus, zMinus, zPlus;

	//здесь разницы времен получения сигнала от маяка до i-го передатчика относительно
	//первого передатчика, в секундах 
	//ti - t0, где ti - задержка i-го передатчика, t0 - задержка 0-го передатчика	
// 	t11 = m_marks[0];
// 	t21 = m_marks[1];
// 	t31 = m_marks[2];
// 	t41 = m_marks[3];
// 	t51 = m_marks[4];
// 	t61 = m_marks[5];
// 	t71 = m_marks[6];
// 	t81 = m_marks[7];
	
	c = 0.3;

	arrAnchVal[0] = m_marks[0];
	arrAnchVal[1] = m_marks[1];
	arrAnchVal[2] = m_marks[2];
	arrAnchVal[3] = m_marks[3];
	arrAnchVal[4] = m_marks[4];
	arrAnchVal[5] = m_marks[5];
	arrAnchVal[6] = m_marks[6];
	arrAnchVal[7] = m_marks[7];
	x = y = z = 0;
	zl = xl = yl = xMinus = xPlus = yMinus = yPlus = zMinus = zPlus = 0;

// 	if (t11 == 0 && t21 == 0 && t31 == 0 && t41 == 0 && t51 == 0 && t61 == 0 && t71 == 0 && t81 == 0)
// 	{
// 		x = XY_DIMENSION / 2; y = XY_DIMENSION / 2; z = Z_DIMENSION / 2;
// 		p3d->x = x; p3d->y = y; p3d->z = z;
// 		return p3d;
// 	}

	a[0] = 0; a[1] = 1; a[2] = 2; a[3] = 3;//индексы приемников, начиная с 0
	l = 0; p = 3;

	//7, begin of the main cycle
	while (p >= 0)
	{
		for (i = 0; i <= 3; i++)
		{
			index_anyJ = getAnyJinAExcludeAi(i);
			if (arrAnchVal[a[i]] != arrAnchVal[index_anyJ])
			{
				s = i;
				goto step15;
			}
		}
		goto step102;
	step15: a_[0] = a[s];
		k = 1;

		//17
		//для каждого j, принадлежащего множеству {a[1]; a[2]; a[3]; a[4]}\{a[s]}
		getarrJinAExcludeAs(s);// get arrJ[] here
		for (int x = 0; x<3; x++)//0..2 потому что множество элементов a[] минус a[s]
		{
			a_[k] = arrJ[x];
			k++;
		}
		//21
		u21 = u[a_[1]] - u[a_[0]];
		u31 = u[a_[2]] - u[a_[0]];
		u41 = u[a_[3]] - u[a_[0]];
		v21 = v[a_[1]] - v[a_[0]];
		v31 = v[a_[2]] - v[a_[0]];
		v41 = v[a_[3]] - v[a_[0]];
		w21 = w[a_[1]] - w[a_[0]];
		w31 = w[a_[2]] - w[a_[0]];
		w41 = w[a_[3]] - w[a_[0]];
		delta21 = u[a_[1]] * u[a_[1]] + v[a_[1]] * v[a_[1]] + w[a_[1]] * w[a_[1]] -
			(u[a_[0]] * u[a_[0]] + v[a_[0]] * v[a_[0]] + w[a_[0]] * w[a_[0]]);
		delta31 = u[a_[2]] * u[a_[2]] + v[a_[2]] * v[a_[2]] + w[a_[2]] * w[a_[2]] -
			(u[a_[0]] * u[a_[0]] + v[a_[0]] * v[a_[0]] + w[a_[0]] * w[a_[0]]);
		delta41 = u[a_[3]] * u[a_[3]] + v[a_[3]] * v[a_[3]] + w[a_[3]] * w[a_[3]] -
			(u[a_[0]] * u[a_[0]] + v[a_[0]] * v[a_[0]] + w[a_[0]] * w[a_[0]]);
		//33
		tau12 = arrAnchVal[a_[0]] - arrAnchVal[a_[1]];
		tau13 = arrAnchVal[a_[0]] - arrAnchVal[a_[2]];
		tau14 = arrAnchVal[a_[0]] - arrAnchVal[a_[3]];
		tau32 = arrAnchVal[a_[2]] - arrAnchVal[a_[1]];
		tau42 = arrAnchVal[a_[3]] - arrAnchVal[a_[1]];

		alpha1 = tau12*u31 - tau13*u21;
		alpha2 = tau12*u41 - tau14*u21;
		beta1 = tau12*v31 - tau13*v21;
		beta2 = tau12*v41 - tau14*v21;

		if ((alpha1*beta2 - alpha2*beta1) == 0)
			goto step102;
		//45
		gamma1 = tau12*w31 - tau13*w21;
		gamma2 = tau12*w41 - tau14*w21;
		g1 = ((c*c*tau12*tau13*tau32)/1e6 + tau12*delta31 - tau13*delta21) / 2;
		g2 = ((c*c*tau12*tau14*tau42) / 1e6 + tau12*delta41 - tau14*delta21) / 2;
		A = (float) (beta1*gamma2 - beta2*gamma1) / (float)(alpha1*beta2 - alpha2*beta1);
		B = (float) (beta2*g1 - beta1*g2) / (alpha1*beta2 - alpha2*beta1);
		C = (float)(alpha2*gamma1 - alpha1*gamma2) / (alpha1*beta2 - alpha2*beta1);
		D = (float)(alpha1*g2 - alpha2*g1) / (alpha1*beta2 - alpha2*beta1);
			//goto step102;
		E = (u21*A + v21*C + w21) / (c*tau12)/1000;
		F = (c*(float)tau12) / 2000 + (2 * (u21*B + v21*D) - (float)delta21) / (2000 * c*tau12);
		//goto step102;
		G = A*A + C*C + 1 - E*E;
		H = 2 * (A*(B - u[a_[0]]) + C*(D - v[a_[0]]) - w[a_[0]] - E*F);
		I = (B - u[a_[0]])*(B - u[a_[0]]) + (D - v[a_[0]])*(D - v[a_[0]]) +
			w[a_[0]] * w[a_[0]] - F*F;
		//goto step102;
		//58
		if (G == 0 && H != 0)
		{
			l++;
			zl = -I / H;
			xl = A*zl + B;
			yl = C*zl + D;
			//
			x += xl;
			y += yl;
			z += zl;
			/////////
			goto step102;
		}
		//65
		if (H == 0 || (((H / (2 * G))*(H / (2 * G)) - I / G)<0))
			goto step102;
		zMinus = -(H / (2 * G)) - sqrt(((H / (2 * G))*(H / (2 * G)) - I / G));
		zPlus = -(H / (2 * G)) + sqrt(((H / (2 * G))*(H / (2 * G)) - I / G));
		//70
		if (zPlus<0 || zMinus>Z_DIMENSION || (zMinus<0 && zPlus>Z_DIMENSION))
			goto step102;
		//73
		xMinus = A*zMinus + B;
		xPlus = A*zPlus + B;
		//75
		if ((xMinus<0 && xPlus<0) || (xMinus>XY_DIMENSION && xPlus>XY_DIMENSION) ||
			(xMinus<0 && xPlus>XY_DIMENSION) || (xPlus<0 && xMinus>XY_DIMENSION))
			goto step102;
		//78
		yMinus = C*zMinus + D;
		yPlus = C*zPlus + D;
		if ((yMinus<0 && yPlus<0) || (yMinus>XY_DIMENSION && yPlus>XY_DIMENSION) ||
			(yMinus<0 && yPlus>XY_DIMENSION) || (yPlus<0 && yMinus>XY_DIMENSION))
			goto step102;
		//83
		if (xMinus<0 || xMinus>XY_DIMENSION || yMinus<0 || yMinus>XY_DIMENSION ||
			zMinus<0 || zMinus>Z_DIMENSION)
		{
			l++;
			xl = xPlus;
			yl = yPlus;
			zl = zPlus;
			//
			x += xl;
			y += yl;
			z += zl;
			/////////
			goto step102;
		}
		//90
		if (xPlus<0 || xPlus>XY_DIMENSION || yPlus<0 || yPlus>XY_DIMENSION ||
			zPlus<0 || zPlus>Z_DIMENSION)
		{
			l++;
			xl = xMinus;
			yl = yMinus;
			zl = zMinus;
			//
			x += xl;
			y += yl;
			z += zl;
			/////////
			goto step102;
		}
		//97
		pt1->x = xMinus;
		pt1->y = yMinus;
		pt1->z = zMinus;
		pt2->x = xPlus;
		pt2->y = yPlus;
		pt2->z = zPlus;
		ptRet->x = 0;
		ptRet->y = 0;
		ptRet->z = 0;

		if (Pair_Analyzing(pt1, pt2, ptRet) == 0)
			goto step102;

		l++;
		Pair_Analyzing(pt1, pt2, ptRet);

		x += ptRet->x;
		y += ptRet->y;
		z += ptRet->z;

	step102: if (a[3] == 7)
		p--;
			 else
				 p = 3;
			 if (p >= 0)
			 {
				 for (int i = 3; i >= p; i--)
					 a[i] = a[p] + i - p + 1;
			 }

	}//end while

	x = x / l;
	y = y / l;
	z = z / l;

	p3d->x = x; p3d->y = y; p3d->z = z;
	return p3d;
}

//получить произвольное время задержки из массива a_[]
int Alg1::getTA_JK(int j, int k)
{
	if (j<0 || j>3 || k<0 || k>3)
	{
		qDebug("getTA_JK: wrong index !");
		return -1;
	}
	else
	{
		return arrAnchVal[a_[j]] - arrAnchVal[a_[k]];
	}
}

//вернуть j ?{a[1]; a[2]; a[3]; a[4]} \ {a[i]}
int Alg1::getAnyJinAExcludeAi(int i)
{
	if (i<0 || i>3)
	{
		qDebug("getAnyJinAExcludeAi: wrong index !");
		return -1;
	}
	else
	{
		for (int l = 0; l<4; l++)
			if (a[l] != a[i])
				return a[l];
	}
}


bool Alg1::getarrJinAExcludeAs(int s)
{
	int indJ = 0;
	if (s<0 || s>3)
	{
		qDebug("getarrJinAExcludeAs: wrong index !");
		return false;
	}
	else
	{
		for (int i = 0; i<4; i++)
			if (a[i] != a[s])
			{
				arrJ[indJ] = a[i];
				indJ++;
			}
		return true;
	}
}