#include <MatrixMath.h>
#include <math.h>
float pi = 3.1415926;
int64_t GPS_data[100][3] = {
{337741241, -843977737,163},
{337741241, -843977737,413},
{337741241, -843977737,664},
{337741241, -843977737,913},
{337741241, -843977737,1163},
{337741241, -843977737,1413},
{337741241, -843977737,1662},
{337741241, -843977737,1912},
{337741241, -843977737,2162},
{337741241, -843977737,2412},
{337741241, -843977737,2663},
{337741241, -843977737,2913},
{337741241, -843977737,3163},
{337741241, -843977737,3412},
{337741241, -843977737,3914},
{337741241, -843977737,4162},
{337741318, -843977737,5163},
{337741432, -843977814,6412},
{337741470, -843977814,6912},
{337741623, -843977814,8413},
{337741661, -843977814,8660},
{337741699, -843977737,8910},
{337741776, -843977814,9660},
{337741852, -843977737,10411},
{337741966, -843977737,11663},
{337742043, -843977737,12160},
{337742119, -843977814,12911},
{337742195, -843977814,13660},
{337742386, -843977814,15409},
{337742462, -843977814,15908},
{337742500, -843977814,16161},
{337742577, -843977814,16908},
{337742767, -843977814,18658},
{337742996, -843977814,20660},
{337743073, -843977814,21156},
{337743149, -843977814,21656},
{337743187, -843977814,21908},
{337743301, -843977814,23156},
{337743378, -843977814,23656},
{337743568, -843977814,25159},
{337743645, -843977814,25906},
{337743759, -843977814,26905},
{337743797, -843977814,27405},
{337743835, -843977814,27655},
{337743988, -843977814,29155},
{337744141, -843977814,30154},
{337744179, -843977814,30404},
{337744179, -843977814,30654},
{337744179, -843977814,30905},
{337744293, -843977814,31653},
{337744370, -843977814,32153},
{337744370, -843977814,32404},
{337744560, -843977814,34156},
{337744598, -843977814,34403},
{337744751, -843977814,35403},
{337744751, -843977814,35902},
{337744942, -843977814,37652},
{337745094, -843977814,38654},
{337745132, -843977814,39151},
{337745323, -843977814,40901},
{337745438, -843977814,42150},
{337745476, -843977814,42400},
{337745552, -843977814,43153},
{337745628, -843977814,43651},
{337745743, -843977814,44401},
{337745781, -843977814,45150},
{337745819, -843977814,45400},
{337746010, -843977814,47150},
{337746086, -843977737,47652},
{337746124, -843977737,47899},
{337746353, -843977737,50148},
{337746468, -843977737,50898},
{337746506, -843977737,51397},
{337746658, -843977737,52647},
{337746735, -843977737,53401},
{337746773, -843977737,53647},
{337746811, -843977737,54147},
{337746887, -843977737,54647},
{337746925, -843977737,54897},
{337747192, -843977737,57396},
{337747269, -843977737,57899},
{337747383, -843977737,58896},
{337747383, -843977737,59147},
{337747421, -843977737,59396},
{337747536, -843977737,60395},
{337747879, -843977737,63144},
{337747917, -843977737,63646},
{337748032, -843977737,64394},
{337748070, -843977737,64894},
{337748222, -843977661,66144},
{337748451, -843977661,68146},
{337748489, -843977661,68392},
{337748718, -843977661,70642},
{337748756, -843977661,70892},
{337748871, -843977661,71642},
{337748909, -843977661,71891},
{337749138, -843977585,74141},
{337749176, -843977585,74641},
{337749405, -843977585,76391},
{337749481, -843977585,77140},
};

int i = 15;
float cosLat = cos((float) GPS_data[0][0]/10000000*pi/180);  
int64_t Rearth = 6378137;
float RcovarianceMatrix[2][2] = {
         {2.21017383364137, 3.51637078682249},
         {3.51637078682249, 13.8032720993553},
  };
  float QcovarianceMatrix[4][4] = {
         {0.01,    0,       0,      0    },
         {0,       0.01,    0,      0    },
         {0,       0,       0.001,  0    },
         {0,       0,       0,      0.001},
  };
float delta_T = ((float) (GPS_data[i][2] - GPS_data[i-1][2])) / 1000; //Time elapsed in seconds
float data[2] = {(float) (GPS_data[i][0]-GPS_data[0][0])*pi/180*Rearth/10000000,(float) (GPS_data[i][1]-GPS_data[0][1])*pi/180*Rearth*cosLat/10000000};
float Xstate[4][1] = {
        {data[0]},
        {data[1]},
        {0.0    },
        {0.0    },
};
float Hmatrix[2][4] = {
      {1,0,0,0},
      {0,1,0,0},
  };
float HmatrixTranspose[4][2] = {
            {1.0, 0.0},
            {0.0, 1.0},
            {0.0, 0.0},
            {0.0, 0.0},
      };  
float PerrorCovariance[4][4] = {
         {0.001, 0,     0,    0   },
         {0,     0.001, 0,    0   },
         {0,     0,     0.02, 0   },
         {0,     0,     0,    0.02},
};

void setup(){  
  Serial.flush();
  Serial.begin(9600);
  delay(10);
  Serial.println("STARTING CALCULATIONS");
  i = i + 1;  
}

void loop(){
  if (i <= 99){
      //Serial.println("**************************************************************");
      /*************Line 76 of Matlab code*************/
      // Prediction Step:
      //Matrix.Print((float*) Hmatrix, 2, 4, "H matrix: ");
      delta_T = ((float) (GPS_data[i][2] - GPS_data[i-1][2])) / 1000; //Time elapsed in seconds
      float Amatrix[4][4] = {
             {1, 0,  delta_T, 0      },
             {0, 1,  0,       delta_T},
             {0, 0,  1,       0      },
             {0, 0,  0,       1      },
             }; 
      
      float nextXstateEstimate[4][1] = {
            {0},
            {0},
            {0},
            {0},
      };
      //Matrix.Print((float*) Amatrix, 4, 4, "Amatrix: ");
      //Matrix.Print((float*) Xstate, 4, 1, "Xstate: ");
      Matrix.Multiply((float*) Amatrix, (float*) Xstate, 4, 4, 1, (float*) nextXstateEstimate); 
      //Matrix.Print((float*) nextXstateEstimate, 4, 1, "Next Estimate X State: ");
      
      /*************Line 78 of Matlab code*************/
      float PerrorCovarianceEstimate[4][4] = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
      };
      float AmatrixTranspose[4][4] = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
      };
      float IntermediateProductMatrix[4][4] = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
      };
      float IntermediateSumMatrix[4][4] = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
      };
      //Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
      //Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
      //Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
      //Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
      //Matrix.Print((float*) PerrorCovarianceEstimate, 4, 4, "PerrorCovarianceEstimate: ");
      
      Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
      Matrix.Print((float*) AmatrixTranspose, 4, 4, "A Transpose: ");
      Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
      Matrix.Print((float*) IntermediateProductMatrix, 4, 4, "IntermediateProductMatrix: ");
      Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
      Matrix.Print((float*) IntermediateSumMatrix, 4, 4, "IntermediateSumMatrix: ");
      Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
      Matrix.Print((float*) QcovarianceMatrix, 4, 4, "QcovarianceMatrix: ");
      Matrix.Print((float*) PerrorCovarianceEstimate, 4, 4, "PerrorCovarianceEstimate: ");
      
      // Correction Step
      /*************Line 81 of Matlab code*************/
      float KalmanGain[4][2] = {
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
      };    
      float IntermediateProductMatrix2[2][4] = {
            {0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
      };
      float IntermediateProductMatrix3[2][2] = {
            {0.0, 0.0},
            {0.0, 0.0}, 
      };
      float IntermediateQuotientMatrix[4][2] = {
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
      };
      float IntermediateSumMatrix2[2][2] = {
            {0.0, 0.0}, 
            {0.0, 0.0},
      };
      //Part in parentheses: (Hmatrix*PerrorCovarianceEstimate*Hmatrix' + RcovarianceMatrix)
      //Matrix.Print((float*) Hmatrix, 2, 4, "Original H: ");
      //Matrix.Print((float*) HmatrixTranspose, 4, 2, "Transpose H: ");
      Matrix.Multiply((float*) Hmatrix, (float*) PerrorCovarianceEstimate, 2,4,4, (float*) IntermediateProductMatrix2);
      Matrix.Multiply((float*) IntermediateProductMatrix, (float*) HmatrixTranspose, 2,4,2, (float*) IntermediateProductMatrix3);
      Matrix.Add((float*) IntermediateProductMatrix3, (float*) RcovarianceMatrix, 2, 2, (float*) IntermediateSumMatrix2);
      //Part outside of parentheses: PerrorCovarianceEstimate*Hmatrix'/
      //Dividing is the same as multiplying by the inverse
      int flag = Matrix.Invert((float*) IntermediateSumMatrix2, 2); //The argument matrix gets inverted; no need to create temp matrix.
      if (flag == 0){
        Serial.println(flag);
      }
      Matrix.Multiply((float*) PerrorCovarianceEstimate, (float*) HmatrixTranspose, 4,4,2, (float*) IntermediateQuotientMatrix);
      Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) IntermediateSumMatrix2, 4,2,2, (float*) KalmanGain);
      
      Matrix.Print((float*) KalmanGain, 4, 2, "Kalman Gain:");
      
      /*************Line 82 of Matlab code*************/
      data[0] = (float) (GPS_data[i][0]-GPS_data[0][0])*pi/180*Rearth/10000000;
      data[1] = (float) (GPS_data[i][1]-GPS_data[0][1])*pi/180*Rearth*cosLat/10000000;     
      float ZkTranspose[2][1] = { //We only need the transposed version of this, so we do it right here.
            {data[0]},
            {data[1]},
            }; //We could have just used the variable "data" instead of Zk, but we do this to be the same as the Matlab code
      //Matrix.Print((float*) ZkTranspose, 2, 1, "Zk:");
      /*************Line 83 of Matlab code*************/
      //Matlab: Xstate = (nextXstateEstimate + KalmanGain*(Zk' - Hmatrix*nextXstateEstimate))';
      float IntermediateProductMatrix4[2][1] = {
            {0.0}, 
            {0.0},
      };
      float IntermediateSubtractionMatrix[2][1] = {
            {0.0}, 
            {0.0},
      };
      //Matrix.Print((float*)Hmatrix, 2, 4, "Hmatrix: ");
      Matrix.Multiply((float*) Hmatrix, (float*) nextXstateEstimate, 2,4,1, (float*) IntermediateProductMatrix4);
      //Matrix.Print((float*) IntermediateProductMatrix4, 2, 1, "IntermediateProductMatrix4: ");
      Matrix.Subtract((float*) ZkTranspose, (float*) IntermediateProductMatrix4, 2, 1, (float*) IntermediateSubtractionMatrix);
      Matrix.Multiply((float*) KalmanGain, (float*) IntermediateSubtractionMatrix, 4,2,1, (float*) IntermediateProductMatrix4); //Reuse intermediate matrix because it has appropriate dimensions
      Matrix.Add((float*) nextXstateEstimate, (float*) IntermediateProductMatrix4, 4,1, (float*) Xstate); //NO NEED TO TRANSPOSE X STATE. WE DID THAT IN MATLAB FOR CONVENIENCE
      //Matrix.Print((float*) IntermediateProductMatrix4, 2, 1, "Temp matrix: ");
      //Matrix.Print((float*) IntermediateSubtractionMatrix, 2, 1, "Temp2 matrix: ");
      //Matrix.Print((float*) Xstate, 4, 1, "Xstate Update: ");
      Matrix.Print((float*) ZkTranspose, 2, 1, "ZK transpose: ");
      Matrix.Print((float*) nextXstateEstimate, 4, 1, "Next X State Estimate: ");
      
      /*************Line 84 of Matlab code*************/
      Matrix.Multiply((float*) KalmanGain, (float*) Hmatrix, 4,2,4, (float*) IntermediateProductMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4
      float IdentityMatrix[4][4] = {
            {1,0,0,0},
            {0,1,0,0},
            {0,0,1,0},
            {0,0,0,1},
      };
      Matrix.Subtract((float*) IdentityMatrix, (float*) IntermediateProductMatrix, 4,4, (float*) IntermediateQuotientMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4.
      Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) PerrorCovarianceEstimate, 4,4,4, (float*) PerrorCovariance);
      //Matrix.Print((float*) PerrorCovariance, 4, 4, "PerrorCovariance: ");
      
      /*************************************************************************************************/
      /*************************************DONE WITH KALMAN FILTER*************************************/
      /*************************************************************************************************/
      
      Serial.print("##");Serial.print(i);Serial.print(",");Matrix.Print((float*) Xstate, 4, 1, "X State: ");
      i = i + 1;
      Serial.print(".....................................................................");
}
  
}


