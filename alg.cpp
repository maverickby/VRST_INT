#include "alg.h"
#include <cmath>
#include <QDebug>
#include <QDateTime>

#define BUFFER_SIZE 128

Alg::Alg()
{
}

Alg::Alg(MainWindow* wnd)
{
    mainWindow = wnd;
    init();
}

void Alg::init()
{
    u[0]=0;    v[0] =0;    w[0] =Z_DIMENSION;	
	u[1]= XY_DIMENSION; v[1] = 0;    w[1] =Z_DIMENSION;
	u[2]=XY_DIMENSION; v[2] =XY_DIMENSION; w[2] =Z_DIMENSION;
	u[3]= 0;    v[3] = XY_DIMENSION; w[3] =Z_DIMENSION;
    u[4]=0;    v[4] =0;    w[4] = Z_LOW_COORD;
	u[5]= XY_DIMENSION; v[5] = 0;    w[5] = Z_LOW_COORD;
	u[6]= XY_DIMENSION; v[6] = XY_DIMENSION; w[6] = Z_LOW_COORD;
	u[7]= 0;    v[7]= XY_DIMENSION; w[7] = Z_LOW_COORD;

	anchor[0].x = 0;            anchor[0].y = 0;            anchor[0].z = Z_DIMENSION;
	anchor[1].x = XY_DIMENSION; anchor[1].y = 0;            anchor[1].z = Z_DIMENSION;
	anchor[2].x = XY_DIMENSION; anchor[2].y = XY_DIMENSION; anchor[2].z = Z_DIMENSION;
	anchor[3].x = 0;            anchor[3].y = XY_DIMENSION; anchor[3].z = Z_DIMENSION;
	anchor[4].x = 0;            anchor[4].y = 0;            anchor[4].z = Z_LOW_COORD;
	anchor[5].x = XY_DIMENSION; anchor[5].y = 0;            anchor[5].z = Z_LOW_COORD;
	anchor[6].x = XY_DIMENSION; anchor[6].y = XY_DIMENSION; anchor[6].z = Z_LOW_COORD;
	anchor[7].x = 0;            anchor[7].y = XY_DIMENSION; anchor[7].z = Z_LOW_COORD;

	ant_delay[0] = 0.00;
	ant_delay[1] = 0.00;
	ant_delay[2] = 0.00;
	ant_delay[3] = 0.00;
	ant_delay[4] = 0.00;
	ant_delay[5] = 0.00;
	ant_delay[6] = 0.00;
	ant_delay[7] = 0.00;

    pt1 = new POINT3D();
    pt1->x=pt1->y=pt1->z=0;
    pt2 = new POINT3D();
    pt2->x=pt2->y=pt2->z=0;
    ptRet = new POINT3D();
    ptRet->x=ptRet->y=ptRet->z=0;
    p3d = new POINT3D();
    p3d->x=p3d->y=p3d->z=0;
	
	ptRawData = new POINT3D();
	ptRawData->x = ptRawData->y = ptRawData->z = 0;

    sync_series = -1;
    adj = 0;
    memset(t_marks, 0, sizeof(t_marks));//clear trash in the array (marks)
    memset(m_marks, 0, sizeof(m_marks));//clear trash in the array
	memset(t_marks_tag0_K_estimate, 0, sizeof(t_marks_tag0_K_estimate));
	memset(K, 0, sizeof(K));
	memset(Kav, 0, sizeof(Kav));
    	    
    memset(tag, 0, sizeof(tag));//clear trash in the array
    memset(a, 0, sizeof(a));//clear trash in the array

	memset(kalmanData, 0, sizeof(kalmanData));
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

	datagramKalmanCount = 0;	
	marks_number = -1;//-1 ������ ��� �������� ����� ������� � 0 � ���������� ���������
}

Alg::~Alg()
{
   delete pt1;
   delete pt2;
   delete ptRet;
   delete p3d;
   delete ptRawData;
}

// bool Alg::ProcessAnchorDatagram(const ANC_MSG* datagram, POINT3D* retPoint)
// {
// 	return false;
// }

// void Alg::process_nav(const ANC_MSG* datagram, POINT3D* retPoint)
// {
// }

// int Alg::prepare_data(int tag)
// {
// 	return 0;
// }

// int Alg::processMarkFilter(int tag_number)
// {
// 	return 0;
// }

// marks filter
// double Alg::mark_filter(int tag, int anc, double d)
// {	
// 	return 0;
// }

//write results to the file
void Alg::WriteToFile(const ANC_MSG* datagram, POINT3D* retPoint, FILE* file)
{
	char buff[BUFFER_SIZE];
	QTime time = QTime::currentTime();

	file_line_string.clear();

	file_line_string = time.toString();
	file_line_string += " ";

    _itoa_s(datagram->sync_n, buff,BUFFER_SIZE, 10);
	file_line_string += buff;
	file_line_string += " ";

	sprintf(buff, "%f", retPoint->x);
	file_line_string += buff;
	file_line_string += " ";
	sprintf(buff, "%f", retPoint->y);
	file_line_string += buff;
	file_line_string += " ";
	sprintf(buff, "%f", retPoint->z);
	file_line_string += buff;
	file_line_string += '\n';

	fwrite(file_line_string.toStdString().c_str(), sizeof(char), file_line_string.length(), file);
}

//write results to the file
void Alg::WriteRawDataToFile(const ANC_MSG* datagram, int anchor_number, double data, FILE* file)
{
	char buff[BUFFER_SIZE];
	QTime time = QTime::currentTime();

	file_line_string_rawdata.clear();

	file_line_string_rawdata = time.toString();
	file_line_string_rawdata += " ";

	_itoa_s(anchor_number, buff, BUFFER_SIZE, 10);
	file_line_string_rawdata += buff;
	file_line_string_rawdata += " ";

	_itoa_s(datagram->sync_n, buff, BUFFER_SIZE, 10);
	file_line_string_rawdata += buff;
	file_line_string_rawdata += " ";

	memset(buff, 0, sizeof(buff));
	sprintf(buff, "%.*g", 16, data);
	file_line_string_rawdata += buff;	
	file_line_string_rawdata += '\n';

	fwrite(file_line_string_rawdata.toStdString().c_str(), sizeof(char), file_line_string_rawdata.length(), file);
}

// distance anchor i to sync anchor 0 in DWT_TIME_UNITS
void Alg::anc_dist(void)
{
    int i;
    for(i = 0; i < ANCHORS_NUMBER; i++)
        anc0dist[i] = sqrt(pow(anchor[i].x - anchor[0].x, 2) +
                           pow(anchor[i].y - anchor[0].y, 2) +
                           pow(anchor[i].z - anchor[0].z, 2)) / SPEED_OF_LIGHT / DWT_TIME_UNITS;
}

// ************  Time marks normalization functions

// to find maximum in m_marks array
double Alg::find_max_m(void)
{
    double a = m_marks[0];
    int i;
    for(i = 1; i < ANCHORS_NUMBER; i ++)
        if(m_marks[i] > a)
            a = m_marks[i];
    return(a);
}

bool Alg::Pair_Analyzing(const POINT3D* pt1,const POINT3D* pt2, POINT3D* ptRet)
{
    int s1,s2;
    double di1,dj1,di2,dj2;
    double tij;

    s1=0;s2=0;

    for(int i=0;i<7;i++)
    {
        for(int j=i+1;i<8;i++)
        {
            di1=sqrt( (u[i]-pt1->x)*(u[i]-pt1->x) + (v[i]-pt1->y)*(v[i]-pt1->y) +
                  (w[i]-pt1->z)*(w[i]-pt1->z) );
            dj1=sqrt( (u[j]-pt1->x)*(u[j]-pt1->x) + (v[j]-pt1->y)*(v[j]-pt1->y) +
                  (w[j]-pt1->z)*(w[j]-pt1->z) );
            tij= arrAnchVal[i]- arrAnchVal[j];
            if( ((tij>=0) && di1<dj1) || (tij<0 && di1>=dj1) )
            {
                s1=1;
                goto step12;
            }
        }
    }

    step12:
    for(int i=0;i<7;i++)
    {
        for(int j=i+1;i<8;i++)
        {
            di2=sqrt( (u[i]-pt2->x)*(u[i]-pt2->x) + (v[i]-pt2->y)*(v[i]-pt2->y) +
                  (w[i]-pt2->z)*(w[i]-pt2->z) );
            dj2=sqrt( (u[j]-pt2->x)*(u[j]-pt2->x) + (v[j]-pt2->y)*(v[j]-pt2->y) +
                  (w[j]-pt2->z)*(w[j]-pt2->z) );
            tij= arrAnchVal[i]- arrAnchVal[j];
            if( ((tij>=0) && di2<dj2) || (tij<0 && di2>=dj2) )
            {
                s2=1;
                goto step22;
            }
        }
    }
    step22:
    if( (s1==1 && s2==1) || (s1==0 && s2==0))
        return false;
    if(s1==1 && s2==0)
    {
        ptRet->x=pt2->x;
        ptRet->y=pt2->y;
        ptRet->z=pt2->z;
        return true;
    }
    if(s1==0 && s2==1)
    {
        ptRet->x=pt1->x;
        ptRet->y=pt1->y;
        ptRet->z=pt1->z;
        return true;
    }
}

