#include <math.h>
#include "PlaneParamEstimator.h"
#include "Matrix.h"

PlaneParamEstimator::PlaneParamEstimator(double delta) : m_deltaSquared(delta*delta) {}
/*****************************************************************************/
/*
 * Compute the plane parameters  [n_x, n_y, n_z, a_x,a_y, a_z]
 */
void PlaneParamEstimator::estimate(std::vector<Node *> &data, std::vector<double> &parameters)
{
	parameters.clear();
	if(data.size() < 3)
		return;

	double AB_x = data[1]->getX() - data[0]->getX();  
	double AB_y = data[1]->getY() - data[0]->getY();
	double AB_z = data[1]->getZ() - data[0]->getZ();

	double AC_x = data[2]->getX() - data[0]->getX();
	double AC_y = data[2]->getY() - data[0]->getY();
	double AC_z = data[2]->getZ() - data[0]->getZ();

	double A = AB_y*AC_z - AB_z*AC_y;
	double B = AB_z*AC_x - AB_x*AC_z;
	double C = AB_x*AC_y - AB_y*AC_x;

	double norm = sqrt(A*A + B*B + C*C);
	
	parameters.push_back(A/norm);
	parameters.push_back(B/norm);
	parameters.push_back(C/norm);
	parameters.push_back(data[0]->getX());
	parameters.push_back(data[0]->getY());	
	parameters.push_back(data[0]->getZ());
}
/*****************************************************************************/
/*
 * Compute the plane parameters  [n_x, n_y, n_z, a_x, a_y, a_z]
 */
void PlaneParamEstimator::leastSquaresEstimate(std::vector<Node *> &data, std::vector<double> &parameters)
{
	double *Matrix[3],*IMatrix[3], Y[3];
	double A, B, C;
	A = B = C = 0.0;

    for (int i = 0;i < 3;i++)
    {
        Matrix[i]  = new double[3];
        IMatrix[i] = new double[3];
    }

    for (int i = 0; i < 3;i++)
    {
        for (int j = 0;j < 3;j++)
        {
            *(Matrix[i] + j) = 0.0;
        }
    }

	for (int i = 0; i < 3; i++)
	{
		Y[i] = 0.0;
	}

    for (int i = 0; i < data.size(); i++)
    {
        *(Matrix[0]) += data[i]->getX()*data[i]->getX();
        *(Matrix[1]) += data[i]->getY()*data[i]->getX();
        *(Matrix[2]) += data[i]->getZ()*data[i]->getX();
        Y[0] -= data[i]->getX();
    }
	for (int i = 0; i < data.size(); i++)
    {
        *(Matrix[0] + 1) += data[i]->getX()*data[i]->getY();
        *(Matrix[1] + 1) += data[i]->getY()*data[i]->getY();
        *(Matrix[2] + 1) += data[i]->getZ()*data[i]->getY();
        Y[1] -= data[i]->getY();
    }

    for (int i = 0; i < data.size(); i++)
    {
        *(Matrix[0] + 2) += data[i]->getX()*data[i]->getZ();
        *(Matrix[1] + 2) += data[i]->getY()*data[i]->getZ();
        *(Matrix[2] + 2) += data[i]->getZ()*data[i]->getZ();
        Y[2] -= data[i]->getZ();
    }

	double d = Determinant(Matrix, 3);

	if (abs(d) < 0.0001)
	{
		printf("\n ¾ØÕóÆæÒì");
//		getchar();

		return;
	}

	Inverse(Matrix, IMatrix, 3, d);
	for (int i = 0; i < 3; i++)
	{
		A += *(IMatrix[0] + i)*Y[i];
		B += *(IMatrix[1] + i)*Y[i];
		C += *(IMatrix[2] + i)*Y[i];
	}

	double norm = sqrt(A*A + B*B + C*C);

	double meanX, meanY, meanZ;
	meanX = meanY = meanZ = 0.0;
	for (int i = 0; i < data.size(); i++)
	{
		meanX += data[i]->getX();
	    meanY += data[i]->getY();
        meanZ += data[i]->getZ();
	}

	meanX /= data.size();
	meanY /= data.size();
	meanZ /= data.size();

	parameters.push_back(A/norm);
	parameters.push_back(B/norm);
	parameters.push_back(C/norm);
	parameters.push_back(meanX);
	parameters.push_back(meanY);
	parameters.push_back(meanZ);

//	printf("\n A = %5.3f, B = %5.3f, C = %5.3f", A, B, C);

	for (int i = 0; i < 3; i++)
	{
		delete [] Matrix[i];
		delete [] IMatrix[i];
	}
}
/*****************************************************************************/
/*
 * Given the plane parameters  [n_x,n_y,n_z,a_x,a_y,a_z] check if
 * [n_x, n_y, n_z] dot [data.x-a_x, data.y-a_y, data.z-a_z] < m_delta
 */
bool PlaneParamEstimator::agree(std::vector<double> &parameters, Node &data)
{
	double signedDistance = parameters[0]*(data.getX() - parameters[3])
		                   +parameters[1]*(data.getY() - parameters[4])
						   +parameters[2]*(data.getZ() - parameters[5]);

	return ((signedDistance*signedDistance) < m_deltaSquared);
}
/*****************************************************************************/
void PlaneParamEstimator::debugTest(std::ostream &out)
{

}
