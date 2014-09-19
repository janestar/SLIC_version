#include"Api.h"

int main(){
	Api api;
	Mat dimg=imread("C:/ee/dImage013.png",0);
	Mat img=imread("C:/ee/cImage013.png");
	int sigma=2;
	int m_spcount=50;
	double m_compactness=10.0;
	api.Do_Depthimage_Repairing(dimg,img,sigma,m_spcount,m_compactness);
	system("pause");
	return 0;

}