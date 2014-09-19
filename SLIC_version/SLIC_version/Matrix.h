#define MAX 10

void Inverse(double *matrix1[],double *matrix2[],int n,double d);
double Determinant(double* matrix[],int n);
double AlCo(double* matrix[],int jie,int row,int column);
double Cofactor(double* matrix[],int jie,int row,int column);

void Inverse(double *matrix1[],double *matrix2[],int n,double d) 
{ 
    int i,j; 
    for(i=0;i<n;i++) 
    {
		matrix2[i]=(double *)malloc(n*sizeof(double)); 
	}
    for(i=0;i<n;i++) 
    {
		for(j=0;j<n;j++) 
        {
			*(matrix2[j]+i)=(AlCo(matrix1,n,i,j)/d); 
		}
	}
} 

double Determinant(double* matrix[],int n)  
{  
    double result=0,temp;  
    int i;  
    if(n==1)  
    {
		result=(*matrix[0]);  
	}
    else  
    {  
        for(i=0;i<n;i++)  
        {  
            temp=AlCo(matrix,n,n-1,i);  
            result+=(*(matrix[n-1]+i))*temp;  
        }  
    }  
    return result;  
}  

double AlCo(double* matrix[],int jie,int row,int column)  
{  
    double result; 
    if((row+column)%2 == 0) 
    {
		result = Cofactor(matrix,jie,row,column);  
	}
    else 
	{
		result=(-1)*Cofactor(matrix,jie,row,column); 
	}

    return result;  
}  

double Cofactor(double* matrix[],int jie,int row,int column)  
{  
    double result;  
    int i,j;  
    double* smallmatr[MAX-1];  
    for(i=0;i<jie-1;i++)  
    {
		smallmatr[i]= new double[jie - 1];
	}
    for(i=0;i<row;i++)  
    {    
		for(j=0;j<column;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i]+j);  
		}
	}
    for(i=row;i<jie-1;i++)  
    {
		for(j=0;j<column;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i+1]+j);  
		}
	}
    for(i=0;i<row;i++)  
    {
		for(j=column;j<jie-1;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i]+j+1);  
		}
	}
    for(i=row;i<jie-1;i++)  
    {
		for(j=column;j<jie-1;j++)  
        {
			*(smallmatr[i]+j)=*(matrix[i+1]+j+1);  
		}
	}
    result = Determinant(smallmatr,jie-1); 
    for(i=0;i<jie-1;i++)
    {
		delete[] smallmatr[i];
	}

    return result;   
}
