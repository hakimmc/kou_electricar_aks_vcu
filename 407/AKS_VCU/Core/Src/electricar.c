#include "electricar.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"



float GpsToDecimalDegrees(const float value, char quadrant){//working

	if(value==0.0){
		return 0;
	}
  float sayi[2];
  int temp_sayi[10];
  int i=0;
  int kalan;
  char* token;
  char nmeaPos[50];
  sprintf(nmeaPos,"%f.6",value);
  token = strtok(nmeaPos,".");
  while (token != NULL){
      sayi[i]= atoi(token);
      token = strtok(NULL,".");
      i++;
  }
  i=0;
  int degree = sayi[0];
  int temp = sayi[1];
     while ( temp > 0 ){
         kalan = temp % 10;
         temp /= 10;
         temp_sayi[i]=kalan;
         i++;
     }
  float min = temp_sayi[5]*10.0+temp_sayi[4]*1.0;
  int carpim_deger=1.0;
  float sec=0.0;
  int j=0;
  for(j=0;j<4;j++){
	 sec+=temp_sayi[j]*carpim_deger;
	 carpim_deger*=10.0;
  }
  min+=sec*pow(10,-4);
  float result;
  result=degree+(min/60.0);
    if(quadrant=='W' || quadrant=='S'){
      result= -1*result;
    }
  return result;
}


float CalculateHeadingVehicle(const float new_lat,const float new_long,const float old_lat,const float old_long){//working

	double angle;
	double starting_degree=0.0;
	double dif_lat = new_lat-old_lat;
	double dif_long = new_long-old_long;
	bool status[4]={false,false,false,false};
	if(dif_lat==0 || dif_long==0){
		return 404;
	}
	else
	{
		angle = atan(abs(new_long-old_long/new_lat-old_lat));
		double direction_angle;

		if(dif_long<0 && dif_lat<0){
			starting_degree=180.0;
			status[0]=true;
			//++
		}
		if(dif_long<0 && dif_lat>0){
			starting_degree=360.0;
			status[1]=true;
			//--
		}
		if(dif_long>0 && dif_lat>0){
			starting_degree=0.0;
			status[2]=true;
			//++
		}
		if(dif_long>0 && dif_lat<0){
			starting_degree=180.0;
			status[3]=true;
			//--
		}
		if(status[0] || status[2]){
			return direction_angle=starting_degree+angle;
		}
		if(status[1] || status[3]){
			return direction_angle=starting_degree+angle;
		}
	}
}


int IS_FLOAT(char* arr){//working
	int i=0;
	while(arr[i]!='.'){
		i++;
		if(arr[i]=='\0'){
			return 0;
		}
	}
	return 1;
	
	/*USAGE 
	IS_FLOAT("55.32")  //%c
	OUTPUT: 1 for "55.32";
	*/
}


char cci2a(char *num){//working
	return (char) atoi(num);
	/*USAGE 
	printf("\r%c\n",ci2a("65"));  //%c
	OUTPUT: A for "65";
	*/
}

char* ccf2a(char *num){//working
	
	char firstascii=(char)(int)atof(num);
	char secondascii=  (char)((int)(atof(num)*100)-(int)(atof(num))*100);
	char temp[3];
	int i=0;
	for(i=0;i<3;i++){
		temp[i] = (i==0) ? firstascii : ((i==1) ? '.' : secondascii);	
	}
	return temp;
	
	/*USAGE 
	printf("\r%s\n",ci2a("65.68")); //%s
	OUTPUT: A.D for "65.68";
	*/
}



int IS_CONTAIN(char* num,char parameter){//working
	int state = strlen(num);
	int i=0;
	while(i+1<=state){
		if(num[i]==parameter){
			return 1;
		}
		i++;
	}
	return 0;
	
	/*USAGE 
	IS_CONTAIN("43.32",'.')
	OUTPUT: 1 for "43.32" & '.';
	*/
}

char* CONVERT_ASCII(char* arr[],int count){//working
	int i=0;
	char tmp[100];
	char int_tmp[count];
	char *float_tmp[count];
	while(i+1<=count){
		if(IS_FLOAT(arr[i])){
			float_tmp[i] = ccf2a(arr[i]);
			printf("girdi1:%s\n",float_tmp[i]);
			strcat(tmp,float_tmp[i]);		
		}
		else{
			int_tmp[i]=cci2a(arr[i]);
			printf("girdi0:%c\n",int_tmp[i]);
			strcat(tmp,&int_tmp[i]);
		}
		i++;
	}
	return tmp;
	/*USAGE 
	char* array1[] = {"65.68","66.54","43","65","23","21"};	
	printf("\n\r%s",CONVERT_ASCII(array,(sizeof(array)/sizeof(array[0]))));
	
	OUTPUT: A.DB.6+A for array1;
	*/
}

int float2twobyte(float f,int i){//working
	if(i==1){
		return (((int)(f*100.0))-(100*((int)f)));
	}
	return (int)(f);
}

/*char* cf2a(float num){//working
	char *str;
	num+=33.33;
	str = (char*)malloc(8);	
	str[0] = (char)(int)(num);
	str[1] = '.';
	str[2] = (char) (((int)(num*100.0))-(100*((int)num)));
	
	return str;
	/*USAGE 
	printf("\r%s\n",cf2a(32.33)); //%s
	OUTPUT: A.D for "65.68";

}*/

char* cf2a(float num,char str[4]){//working
	num+=33.33;
	str[0] = (char)(int)(num);
	str[1] = '0';
	str[2] = (char) (((int)(num*100.0))-(100*((int)num)));
	str[3] = '\0';

	return str;
	/*USAGE
	printf("\r%s\n",ci2a("65.68")); //%s
	OUTPUT: A.D for "65.68";
	*/
}

char ci2a(int num){//working
	return (char) (num+33);
	/*USAGE 
	printf("\r%c\n",ci2a(32));  //%c
	OUTPUT: A for "65";
	*/
}

uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

int calculate_angle(int adc_val,int max_adc_val,char which_variable){
	/*
	 *  which_variable 'a' for angle
	 *  which_variable 'd' for direction
	 *
		0/-180 left
		-180/-360 right
		-360/-450 left
		0/180 right
		180/360 left
		360/450 right
	*/
	int angle = MAP(adc_val,0,max_adc_val,0,900)-450;
	if(which_variable=='a'){
		return angle;
	}
	else if(which_variable=='d'){
	if(angle==0 || angle==-180 || angle==-360 || angle==180 || angle==360){
		return IDLE;
	}else if(-180<angle && angle<0){
		return LEFT;
	}else if(-360<angle && angle<-180){
		return RIGHT;
	}else if(-450<=angle && angle<-360){
		return LEFT;
	}else if(0<angle && angle<180){
		return RIGHT;
	}else if(180<angle && angle<360){
		return LEFT;
	}else if(360<angle && angle<=450){
		return RIGHT;
	}else{
		return ERR;
	}	
	}
}

int binarray_to_decimal(uint8_t array[8]){
	int cnt=0;
	for(int i=0;i<8;i++){
		cnt+= (array[i]*(1<<7-i));
	}
	return cnt;
}

void decimal_to_binarray(int dec,uint8_t dest_array[8]){
    for(int i=0;i<8;i++){
        dest_array[i] = dec%2;
        dec/=2;
    }
}

void NEXTION_SEND(UART_HandleTypeDef huart,int what_do_you_want,char* ID,int variable,char myMessage[50]){
	//0 for vis
	//1 for value
	//2 for txt
	int lenx;
	uint8_t cmdEnd[3] = { 0xFF, 0xFF, 0xFF };
	int leny;
	char arr[8];
	char x[5];

	switch (what_do_you_want) {
		case 0:
			lenx = sprintf(myMessage, "vis %s,%d", ID,variable);
			break;
		case 1:
			lenx = sprintf(myMessage, "%s.val=%d", ID,variable);
			break;
		case 2:
			lenx = sprintf(myMessage, "%s.txt=\"", ID);
			leny = sprintf(x, "%d", variable);
			strcat(myMessage,x);
			strcat(myMessage,"\"");
			break;
		default:
			break;
	}
	HAL_UART_Transmit(&huart,(uint8_t*)myMessage , strlen(myMessage), 1000);
	HAL_UART_Transmit(&huart, cmdEnd, 3, 1000);
}

int CAN_Send_With_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, uint8_t txmessage[], uint32_t *pTxMailbox,CAN_RxHeaderTypeDef *rxHeader,int timeout){
	int t_out=0;
	int return_value;
	uint32_t id=txHeader->StdId;
	while(1){
		if(!(HAL_CAN_AddTxMessage(hcan, txHeader, txmessage, pTxMailbox))){

		}
		uint8_t rxData[1];

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, rxData);
		if(rxData[0]==id){
			return_value = HAL_OK;
			break;
		}
		t_out++;
		HAL_Delay(1);
		if(t_out==timeout){
			return_value = HAL_TIMEOUT;
			break;
		}
	}
	return return_value;
}

int CAN_Read_With_Feedback(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *txHeader, uint8_t txmessage[], uint32_t *TxMailbox,CAN_RxHeaderTypeDef *rxHeader,uint8_t RxData[8],int function_priority,void func()){


	/*do something..*/
	int state1,state2;
	switch (function_priority) {
		case 0:
			func();
			state1=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, RxData);
			state2=HAL_CAN_AddTxMessage(hcan, txHeader,(uint8_t *) rxHeader->StdId, TxMailbox);
			break;
		case 1:
			state1=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, RxData);
			func();
			state2=HAL_CAN_AddTxMessage(hcan, txHeader, (uint8_t *) rxHeader->StdId, TxMailbox);
			break;
		case 2:
			state1=HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, RxData);
			state2=HAL_CAN_AddTxMessage(hcan, txHeader, (uint8_t *) rxHeader->StdId, TxMailbox);
			func();
			break;
	}
	// return 1 is OK, return 0 is ERROR
	return (!state1&!state2);
}


