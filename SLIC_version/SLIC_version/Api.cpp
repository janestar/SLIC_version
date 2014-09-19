#include"Api.h"


//the interface of out algorithm

void Api:: Do_Depthimage_Repairing(cv::Mat& depth,cv::Mat& color,int sigma,int m_spcount,double m_compactness){
	    Mat depth2;
		depth.copyTo(depth2);
	    Holefilling hole;
	    hole.setMat(depth);
	    hole.BFS();
		cv::imwrite("C:/pic/d_holefilling.png",depth);
		Mat cimg;
	    cimg.create(color.rows,color.cols,color.type());
	    cv::bilateralFilter(color,cimg,5,40,10);
		cv::imwrite("C:/pic/c_bilateral.png",cimg);

		int width=cimg.size().width;
	    int height=cimg.size().height;
	    int sz=width*height;
	    int* labels = new int[sz];
	    int numlabels(0);

	  //to ensure the size you set is meaningful
	  if(m_spcount < 10 || m_spcount > sz/4) m_spcount = sz/200;//i.e the default size of the superpixel is 200 pixels
	  if(m_compactness < 1.0 || m_compactness > 80.0) m_compactness = 20.0;
	
	    SLIC slic;
	    slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(cimg, width, height, labels, numlabels, m_spcount, m_compactness);
	    slic.DrawContoursAroundSegments(depth, labels, width, height, 0,numlabels);
	    cv::imwrite("C:/pic/d_SLIC.png",depth);
	   
	   vector<vector<Node>> vec=slic.getVector2();
	   vector<vector<Node>> vec_after_ransac=slic.getVector3();
	   vector<vector<Node>>::size_type size=vec.size();
	   vec_after_ransac.resize(size);
	   vector<vector<Node>>::iterator it;
	   vector<vector<Node>>::iterator it_;
	   vector<Node>::iterator it_contour;
	   //  cout<<vec.size()<<endl;

	   //for each segment,we use the RANSAC algorithm to eliminate the outliers
	   for(it=vec.begin(),it_=vec_after_ransac.begin();it!=vec.end()&&it_!=vec_after_ransac.end();it++,it_++){
		vector<Node>::iterator it2;
		std::vector<double> planeParameters;
	    PlaneParamEstimator lpEstimator(0.5); 
	    int numForEstimate = 3;
		double usedData = Ransac<Node,double>::compute(planeParameters, &lpEstimator, *it, numForEstimate,*it_); 
	    double resultA = planeParameters[0];
	    double resultB = planeParameters[1];

	    double resultC = planeParameters[2];
	    double resultX = planeParameters[3]; 
	    double resultY = planeParameters[4]; 
        double resultZ = planeParameters[5]; 

	std::cout<<"RANSAC plane parameters [A, B, C, X0, Y0, Z0]\n\t [ "<<planeParameters[0]<<", "<<planeParameters[1]<<", "
	                                                        	<<planeParameters[2]<<", "<<planeParameters[3]<<", "
		                                                        <<planeParameters[4]<<", "<<planeParameters[5]<<", ";
	std::cout<<"\tPercentage of points which were used for final estimate: "<<usedData<<std::endl;
	
	}
	   

    //apply plane fitting algorithm to estimate a plane that represents the depth within this segment best
	vector<vector<Node>>::iterator it_2;
	vector<vector<Node>>::iterator it_3;
	int num=0;
	
	for(it_2=vec_after_ransac.begin(),it_3=vec.begin();it_2!=vec_after_ransac.end()&&it_3!=vec.end();it_2++,it_3++){
	
		vector<Node>::iterator ite;
		float A[3][3]={
			{0.0,0.0,0.0},
			{0.0,0.0,0.0},
			{0.0,0.0,0.0}
		};
		float tx=0.0;
		float ty=0.0;
		float tz=0.0;
		int cnt=0;
		char numName[50];
		
		
		sprintf_s(numName,"abc%.2d.txt",num);
		ofstream os(numName);
		 for(ite=it_2->begin();ite!=it_2->end();ite++){
			 float xx=ite->getX();
			 float yy=ite->getY();
			 float zz=ite->getZ();
			 os<<xx<<" "<<yy<<" "<<zz<<" "<<endl;
			 tx+=xx;
			 ty+=yy;
			 tz+=zz;
			 cnt++;
			     }
		 os.close();
		 num++;
		 float tx2=tx/cnt;
		 float ty2=ty/cnt;
		 float tz2=tz/cnt;
		 float tx3=0.0;
		 float ty3=0.0;
		 float tz3=0.0;
		  for(ite=it_2->begin();ite!=it_2->end();ite++){
			 float xx=ite->getX();
			 float yy=ite->getY();
			 float zz=ite->getZ();
			 tx3=xx-tx2;
			 ty3=yy-ty2;
			 tz3=zz-tz2;
			A[0][0]+=tx3*tx3;
			A[0][1]+=tx3*ty3;
			A[0][2]+=tx3*tz3;
			A[1][0]+=tx3*ty3;
			A[1][1]+=ty3*ty3;
			A[1][2]+=ty3*tz3;
			A[2][0]+=tx3*tz3;
			A[2][1]+=ty3*tz3;
			A[2][2]+=tz3*tz3;

			     }
		  Mat ma=Mat(3,3,CV_32FC1,A);
		  float B[3][3]={{0.0,0.0,0.0},
	               {0.0,0.0,0.0},
	               {0.0,0.0,0.0}};

	      Mat mb=Mat(3,3,CV_32FC1,B);
	      float C[3]={0.0,0.0,0.0};
	      Mat mc=Mat(3,1,CV_32FC1,C);
		  bool f=cv::eigen(ma,mc,mb);
		   cout<<mb.row(0)<<endl;
		    cout<<mb.row(1)<<endl;
		    cout<<mb.row(2)<<endl;
			cout<<mc.at<float>(0,0)<<endl;
			cout<<mc.at<float>(1,0)<<endl;
			cout<<mc.at<float>(2,0)<<endl;

		  float a=0.0;
		  float b=0.0;
		  float c=0.0;
		  float dd=0.0;
		  float norm=0.0;
		  if(f){
			   a=mb.row(2).at<float>(0,0);
			   b=mb.row(2).at<float>(0,1);
			   c=mb.row(2).at<float>(0,2);
			   norm=sqrt(a*a+b*b+c*c);
			   a=a/norm;
			   b=b/norm;
			   c=c/norm;
		       dd=tx2*a+ty2*b+tz2*c;

			   for(ite=it_3->begin();ite!=it_3->end();ite++){
			  // for(ite=it_2->begin();ite!=it_2->end();ite++){  
				   int x=ite->getX();
				   int y=ite->getY();
				   int z=depth.at<uchar>(y,x);
				   int temp_z=z;
				   if(c){
					  temp_z=saturate_cast<uchar>((dd-(a*x+b*y))/c);

				      //depth.at<uchar>(y,x)=temp_z;
				   }
				  if(abs(z-temp_z)>sigma){
					   depth2.at<uchar>(y,x)=temp_z;
				   
				   }
			   
			   }
		  }

	}


	cv::imwrite("C:/pic/final.png",depth2);


	}




