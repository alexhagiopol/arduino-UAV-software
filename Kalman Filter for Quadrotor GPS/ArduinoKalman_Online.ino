#include "NazaDecoderLib.h"
#include <MatrixMath.h>
#include <math.h>
float pi = 3.1415926;

uint32_t currTime, attiTime;
unsigned long time;

int i = 1;
float cosLat = 0.0;  
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
float delta_T = 0; //((float) (GPS_data[i][2] - GPS_data[i-1][2])) / 1000; //Time elapsed in seconds
float data[2] = {0,0}; //{(float) (GPS_data[i][0]-GPS_data[0][0])*pi/180*Rearth/10000000,(float) (GPS_data[i][1]-GPS_data[0][1])*pi/180*Rearth*cosLat/10000000};
float Xstate[4][1] = {
        {data[0]},
        {data[1]},
        {0.0    },
        {0.0    },
};
int64_t GPS_data[1][3] = {
        {0,0,0},
};

int64_t prevGPS_data[1][3] = {
        {0,0,0},
};
int64_t firstGPS_data[1][3] = {
        {0,0,0},
};

void setup()
{    
  Serial.begin(9600); //Begin using USB serial port; not serial pin
  Serial3.begin(115200); //Use serial pin 3; 115200 is Naza GPS baud rate
  Serial.println("STARTING"); 
  time = millis();  
}

void loop()
{
  if(Serial3.available())
  {
    uint8_t decodedMessage = NazaDecoder.decode(Serial3.read()); 
    switch (decodedMessage)
    {
      case NAZA_MESSAGE_GPS:
        time = millis();
        prevGPS_data[0][0] = GPS_data[0][0];
        prevGPS_data[0][1] = GPS_data[0][1];
        prevGPS_data[0][2] = GPS_data[0][2];
        GPS_data[0][0] = NazaDecoder.getLat();
        GPS_data[0][1] = NazaDecoder.getLon();
        GPS_data[0][2] = (uint64_t) time;           
        //Serial.print(NazaDecoder.getLat());
        //Serial.print(",");
        //Serial.print(NazaDecoder.getLon());
        //Serial.print(",");
        //Serial.print(time);
        //Serial.print(",\n");
        
        if ((GPS_data[0][0] > 0) && (GPS_data[0][1] < 0)){ //Test for valid GPS data in Continental U.S.
          //KALMAN OPERATIONS BELOW
          if (i == 1){ //Initialize cosLat
            
            firstGPS_data[0][0] = GPS_data[0][0];
            firstGPS_data[0][1] = GPS_data[0][1];
            firstGPS_data[0][2] = GPS_data[0][2]; 
            cosLat = cos((float) firstGPS_data[0][0]/10000000*pi/180);
            //Serial.println("COS LAT DONE");
          }
              
          delta_T = ((float) (GPS_data[0][2] - prevGPS_data[0][2])) / 1000; //Time elapsed in seconds
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
          //Matrix.Print((float*) AmatrixTranspose, 4, 4, "A Transpose: ");
          Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
          //Matrix.Print((float*) IntermediateProductMatrix, 4, 4, "IntermediateProductMatrix: ");
          Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
          //Matrix.Print((float*) IntermediateSumMatrix, 4, 4, "IntermediateSumMatrix: ");
          Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
          //Matrix.Print((float*) QcovarianceMatrix, 4, 4, "QcovarianceMatrix: ");
          //Matrix.Print((float*) PerrorCovarianceEstimate, 4, 4, "PerrorCovarianceEstimate: ");
          
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
          Matrix.Multiply((float*) IntermediateProductMatrix2, (float*) HmatrixTranspose, 2,4,2, (float*) IntermediateProductMatrix3);
          Matrix.Add((float*) IntermediateProductMatrix3, (float*) RcovarianceMatrix, 2, 2, (float*) IntermediateSumMatrix2);
          //Part outside of parentheses: PerrorCovarianceEstimate*Hmatrix'/
          //Dividing is the same as multiplying by the inverse
          int flag = Matrix.Invert((float*) IntermediateSumMatrix2, 2); //The argument matrix gets inverted; no need to create temp matrix.
          if (flag == 0){
            Serial.println("CANT INVERT");
          }
          Matrix.Multiply((float*) PerrorCovarianceEstimate, (float*) HmatrixTranspose, 4,4,2, (float*) IntermediateQuotientMatrix);
          Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) IntermediateSumMatrix2, 4,2,2, (float*) KalmanGain);
          
          //Matrix.Print((float*) KalmanGain, 4, 2, "Kalman Gain:");
          
          /*************Line 82 of Matlab code*************/
          data[0] = (float) (GPS_data[0][0]-firstGPS_data[0][0])*pi/180*Rearth/10000000;
          data[1] = (float) (GPS_data[0][1]-firstGPS_data[0][1])*pi/180*Rearth*cosLat/10000000;     
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
          float IntermediateProductMatrix5[4][1] = {
                {0.0}, 
                {0.0},
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
          Matrix.Multiply((float*) KalmanGain, (float*) IntermediateSubtractionMatrix, 4,2,1, (float*) IntermediateProductMatrix5); //Reuse intermediate matrix because it has appropriate dimensions
          Matrix.Add((float*) nextXstateEstimate, (float*) IntermediateProductMatrix5, 4,1, (float*) Xstate); //NO NEED TO TRANSPOSE X STATE. WE DID THAT IN MATLAB FOR CONVENIENCE
          //Matrix.Print((float*) IntermediateProductMatrix4, 2, 1, "Temp matrix: ");
          //Matrix.Print((float*) IntermediateSubtractionMatrix, 2, 1, "Temp2 matrix: ");
          //Matrix.Print((float*) Xstate, 4, 1, "Xstate Update: ");
          //Matrix.Print((float*) ZkTranspose, 2, 1, "ZK transpose: ");
          //Matrix.Print((float*) nextXstateEstimate, 4, 1, "Next X State Estimate: ");
          
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
          
          Serial.print(i);Serial.print(",");Serial.print(Xstate[0][0]);Serial.print(",");Serial.print(Xstate[1][0]);Serial.print(",");Serial.print(Xstate[2][0]);Serial.print(",");Serial.print(Xstate[3][0]);Serial.print(",");Serial.print(ZkTranspose[0][0]);Serial.print(",");Serial.print(ZkTranspose[1][0]);Serial.print(",");Serial.print((int32_t) GPS_data[0][0]);Serial.print(",");Serial.print((int32_t) GPS_data[0][1]);Serial.print(",");Serial.println((int32_t) GPS_data[0][2]);
          //Serial.print("Current GPS ");Serial.print((int32_t) GPS_data[0][0]);Serial.print(",");Serial.print((int32_t) GPS_data[0][1]);Serial.print(",");Serial.print((int32_t) GPS_data[0][2]);Serial.println(",");
          //Serial.print("First GPS ");Serial.print((int32_t) firstGPS_data[0][0]);Serial.print(",");Serial.print((int32_t) firstGPS_data[0][1]);Serial.print(",");Serial.print((int32_t)firstGPS_data[0][2]);Serial.println(",");
          //Serial.println((float) ((GPS_data[0][0]-firstGPS_data[0][0])*pi/180*Rearth/10000000));
          //Serial.println((float) ((GPS_data[0][1]-firstGPS_data[0][1])*pi/180*Rearth/10000000*cosLat));
          //Serial.println(cos((float) firstGPS_data[0][0]/10000000*pi/180));
          i = i + 1;
          //data[0] = (float) (GPS_data[0][0]-firstGPS_data[0][0])*pi/180*Rearth/10000000;
          //data[1] = (float) (GPS_data[0][1]-firstGPS_data[0][1])*pi/180*Rearth*cosLat/10000000;  
        }       
        break;       
    }      
  }
}

