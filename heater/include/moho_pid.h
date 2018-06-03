extern u16 PID_Calculate(void) ;
 
/***********************************************
         函数功能：PID参数Kp的计算 
************************************************/ 
extern float fuzzy_kp(float e, float ec);				//e,ec，表示误差，误差变化率

/***********************************************
         函数功能：PID参数Ki的计算 
************************************************/ 
extern float fuzzy_ki(float e, float ec);				        

/***********************************************
         函数功能：PID参数Kd的计算 
************************************************/ 
extern float fuzzy_kd(float e, float ec);


extern u16 mohu_pid(u16 setTemp,u16 curTemp);
