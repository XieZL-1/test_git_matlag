#include <stdio.h> 
#include <string.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
#include "mex.h"

static unsigned char  crc8(unsigned char data, unsigned int crc) {
	unsigned char  crc8 = (0x107 & 0xFF);
    crc ^= data;    //前一字节计算CRC后的结果异或上后一字节，再次计算CRC
	for (int i=0; i<8; i++){
		if (crc & 0x80){
		   crc <<= 1;
		   crc ^= crc8;
		}
		else{
		   crc <<= 1;
		}
	}
	return crc;
}

unsigned char  Calculate_Crc8(unsigned char* buf, int len){
    unsigned char  crc = 0x00;
    while(len--)
	{
        crc = crc8(*buf, crc);
        if(len>0x00){
            buf++;
        }
    }    
	return crc;
}
void mexFunction(int nlhs, mxArray *plhs[], 
                 int nrhs, const mxArray *prhs[]){
    //nlhs代表的是输出参数的个数
    //plhs是一个指针数组，里面的指针指向mxArray类型，每一个指针指向一个输出
    //nrhs代表的是输入参数的个数
    //prhs是一个指针数组，里面的指针指向mxArray类型，每一个指针指向一个输入


    // int ncols = mxGetN(prhs[0]);   //获得矩阵的列
    // printf("%d_%d\n", mrows, ncols);  //打印行和列
    // uint8_t parms[mrows][ncols] = {0};
    // parms.resize(mrows);  //初始化
    // for(int i = 0; i < mrows; i++){
    //     parms[i].resize(ncols);
    // }
 
    // for(int i = 0; i < mrows; i++){
    //     for(int j = 0; j < ncols; j++){
    //         parms[i][j] = dataCursor[j * mrows + i]; //拷贝矩阵的元素到vector of vector
    //     }
    // }





    

    char *dataCursor;
    dataCursor =  (char *)mxGetPr(prhs[0]); //得到输入矩阵的第一个元素的指针
    int mrows = mxGetM(prhs[0]);   //获得矩阵的行
    int ncols = mxGetN(prhs[0]);   //获得矩阵的列

    char **parms = (char**)malloc(sizeof(char*) * mrows);  //sizeof(int*),不能少*，一个指针的内存大小，每个元素是一个指针。
    for (int i = 0;i < mrows;i++)
    {
        parms[i] = (char*)malloc(sizeof(char) * ncols);
    }
    // char parms[][];
    char *dataCursor1;
    // const int *cellColPtr = mxGetDimensions(prhs[1]);
    mxArray *cellOfStr;
    char *chTmp;
    unsigned long* crc_32 ;
    plhs[0] = mxCreateDoubleMatrix( mrows, 1, mxREAL);//第一个输出是一个5*6的矩阵
    double *z;
    z = mxGetPr(plhs[0]);//获得矩阵的第一个元素的指针
    // int error[mrows] = {0};
    int  e = 0;
    dataCursor1 = (char *)mxGetPr(prhs[0]); //得到输入矩阵的第一个元素的指针
    for(int i = 0; i < mrows; i++){
        for(int j = 0; j < ncols; j++){
            parms[i][j] = dataCursor1[j * mrows + i]; //拷贝矩阵的元素到vector of vector
        }
    }
    int length = mxGetScalar(prhs[2]);


    for (int i = 0 ; i<mrows;i++)
    {
      cellOfStr = mxGetCell(prhs[1], i); //当然cell里面可以是字符串
      chTmp = mxArrayToString(cellOfStr);//这里输出的就是字符串
      char *ptr;
      unsigned char ret;
      ret = strtoul(chTmp, &ptr, 16);
      // unsigned long chl = (unsigned long)chTmp;
      // printf("%s\n", chTmp);
      // crc_32 = (unsigned long *)chTmp;
      // dataCursor1 = (char *)mxGetPr(prhs[0]); //得到输入矩阵的第一个元素的指针
      // unsigned long CRC = ~CalculateCRC32((unsigned char*)parms[i],70);    //这里是数据

      unsigned char CRC = Calculate_Crc8((unsigned char*)parms[i],length);

      // unsigned long CRC = CalculateBlockCRC32(296, (unsigned char*)parms[i]);
      // printf("%lx\n", ret);
      if (ret != CRC) 
      {
        z[e] = i+1;
        e++;
      } 
    }
    free(parms);














    return;
                 }

    

 
// 　　vector<vector<double> > array2d;
//     double *z;
//     plhs[0] = mxCreateDoubleMatrix( 5, 6, mxREAL);//第一个输出是一个5*6的矩阵
//     z = mxGetPr(plhs[0]);//获得矩阵的第一个元素的指针
//     array2d.resize(5);
//     int ii = 0;
//     for(int i = 0; i < 5; i++){
//         for(int j = 0; j < 6; j++){
//             z[i*6 + j] = ii; //指针访问矩阵是列优先的，请自己循环程序和分析输出结果
//             ii++;
//         }
//     }








//   char *dataCursor;
// 	unsigned long  crc_32 = mxGetScalar(prhs[1]); 
// 	double *y;
// 	//char *crc_32;
// 	//crc_32 =  (char *)mxGetPr(prhs[1]); 
// 	dataCursor = (char *)mxGetPr(prhs[0]); //得到输入矩阵的第一个元素的指针
// 	int mrows = mxGetM(prhs[0]);    //获得矩阵的行
// 	//for (int i = 0;i<mrows;i++)
// //	{
//  unsigned long CRC = ~CalculateCRC32((unsigned char*)dataCursor,296);
// 		// unsigned long CRC = CalculateBlockCRC32(296, (unsigned char*)dataCursor);
// 	//	dataCursor = dataCursor+296;
		

//  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL); //让第一个输出参数指向一个1*1的矩阵
//  y = mxGetPr(plhs[0]); //获得矩阵的第一个元素的指针

//    if (crc_32 == CRC)  *y =1;
//    else *y = 0;
		//cout <<"string:"<< CRC << endl;
//	}




    //vector<vector<uint8_t> > parms;

   // int mrows = mxGetM(prhs[0]);   //��þ������
   // int ncols = mxGetN(prhs[0]);   //��þ������
//	char  parms[mrows][ncols];
//	parms.resize(mrows);  //��ʼ��
//	for(int i = 0; i < mrows; i++){
 //       parms[i].resize(ncols);		printf("%s %d\n", "CRC32 in Hex is: ", *dataCursor);
		//	cout <<"string:"<< CRC << endl;
 //   }
//	for(int i = 0; i < mrows; i++){
   //     for(int j = 0; j < ncols; j++){
  //          parms[i][j] = dataCursor[j * mrows + i]; //���������Ԫ�ص�vector of vector
    //    }
  //  }


// }



//int main() { 

   // char *msgBlock = "AA44AA45"; 
 //   unsigned long CRC = CalculateBlockCRC32(strlen(msgBlock), (unsigned char*)msgBlock);
   // printf("%s %lx\n", "CRC32 in Hex is: ", CRC);
  // cout <<"string:"<< CRC << endl;
  //  system("pause");
//
//}