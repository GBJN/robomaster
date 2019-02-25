
/* Includes ------------------------------------------------------------------*/

#include "PID.h"




PID_Regulator_t RBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;//位置环要不要加以后再说
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t AMRotatePID = CHASSIS_MOTOR_SPEED_PID_DEFAULT; 

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3


int deltaKpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
												 {PB,PB,PM,PS,PS,ZO,NS},
												 {PM,PM,PM,PS,ZO,NS,NS},
												 {PM,PM,PS,ZO,NS,NM,NM},
												 {PS,PS,ZO,NS,NS,NM,NM},
												 {PS,ZO,NS,NM,NM,NM,NB},
												 {ZO,ZO,NM,NM,NM,NB,NB}};

int deltaKiMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
												 {NB,NB,NM,NS,NS,ZO,ZO},
												 {NB,NM,NS,NS,ZO,PS,PS},
												 {NM,NM,NS,ZO,PS,PM,PM},
												 {NM,NS,ZO,PS,PS,PM,PB},
												 {ZO,ZO,PS,PS,PM,PB,PB},
												 {ZO,ZO,PS,PM,PM,PB,PB}};

int deltaKdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},
												 {PS,NS,NB,NM,NM,NS,ZO},
												 {ZO,NS,NM,NM,NS,NS,ZO},
												 {ZO,NS,NS,NS,NS,NS,ZO},
												 {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
												 {PB,NS,PS,PS,PS,PS,PB},
												 {PB,PM,PM,PM,PS,PS,PB}};



float	EC = 0;
float	E = 0;
float E_Index = 0;
float EC_Index = 0;
	 
void PID_Reset(PID_Regulator_t *pid)
{
	pid->err[0]=0;//err0作为误差积分
	pid->err[1]=0;//err1作为本次误差
	pid->err[2]=0;//err2作为上次误差
	pid->err[3]=0;//err2作为上上次误差
	pid->output=0;
}





void PID_Calc(PID_Regulator_t *pid)
{
	
	pid->err[3]=pid->err[2];//只有在增量式pid中才用到了上上次误差及err3
	pid->err[2]=pid->err[1];//上次误差
	pid->err[1]=pid->ref-pid->fdb;//本次误差
	pid->err[0]+=pid->err[1];//误差积分
	//VAL_LIMIT(pid->ki,-pid->componentKiMax,pid->componentKiMax);	//抗饱和积分//这是错的，明明该限制output的变成了限制ki了，删掉他
	if(pid->type == POSITION_PID)
	{
		pid->output_kp = pid->kp*pid->err[1];//比例部分
		pid->output_ki = pid->ki*pid->err[2];//积分部分
		pid->output_kd = pid->kd*(pid->err[1]-pid->err[0]);	//最后一个是误差微分部分的输出
		
		VAL_LIMIT(pid->output_kp,-pid->output_kpMax,pid->output_kpMax);
		VAL_LIMIT(pid->output_ki,-pid->output_kiMax,pid->output_kiMax);//一般在这项做限制作为抗饱和积分//上下两个加不加以后看
		VAL_LIMIT(pid->output_kd,-pid->output_kdMax,pid->output_kdMax);
		pid->output = pid->output_kp + pid->output_ki + pid->output_kd;
	}
	else if(pid->type == DELTA_PID)
	{
		pid->output = pid->kp*(pid->err[1]-pid->err[2])+pid->ki*pid->err[1]+pid->kd*(pid->err[1] - 2*pid->err[2]+pid->err[3]);//增量式pid，只使用于有记忆功能的器件，如步进电机等
	}
	else if(pid->type == VAGUE_PID)
	{

		static float kp_delta = 0;
		static float ki_delta = 0;
		static float kd_delta = 0;
		
		EC = (pid->err[2]-pid->err[1]);//误差变化率,这样的单位是误差/ms
		E = 	pid->err[1];
		

								//*6是为了化成0-6的整数
//		E_Index = E*6/pid->refmax - (E*6/pid->refmax - 1);//靠左隶属度,另一个是1-E_Index
//		EC_Index = EC*6/pid->ecmax - (EC*6/pid->ecmax - 1);
		
//		kp_delta = pid->kp * (deltaKpMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)]*(1-E_Index)*(1-EC_Index)\
													 +deltaKpMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+3)]*(EC_Index)*(1-E_Index)\
													 +deltaKpMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+2)]*(1-EC_Index)*(E_Index)\
													 +deltaKpMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+2)]*(EC_Index)*(E_Index);
		
		
//		ki_delta = pid->kp * (deltaKiMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)]*(1-E_Index)*(1-EC_Index)\
													 +deltaKiMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+3)]*(EC_Index)*(1-E_Index)\
													 +deltaKiMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+2)]*(1-EC_Index)*(E_Index)\
													 +deltaKiMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+2)]*(EC_Index)*(E_Index);
													 
													 
//		kp_delta = pid->kp * (deltaKdMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)]*(1-E_Index)*(1-EC_Index)\
													 +deltaKdMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+3)]*(EC_Index)*(1-E_Index)\
													 +deltaKdMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+2)]*(1-EC_Index)*(E_Index)\
													 +deltaKdMatrix[(int8_t)(E*6/pid->refmax+2)][(int8_t)(EC*6/pid->ecmax+2)]*(EC_Index)*(E_Index);

		
		
		
		
		//这个int8_t 强制转换成比他大的那个整数
//		ki_delta = pid->ki * deltaKiMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)];
//		kd_delta = pid->kd * deltaKdMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)];
		
		
		
		
		pid->kp = pid->kp+kp_delta;
		pid->ki = pid->ki+ki_delta;
		pid->kd = pid->kd+kd_delta;

		pid->output_kp = pid->kp*pid->err[1];//比例部分
		pid->output_ki = pid->ki*pid->err[2];//积分部分
		pid->output_kd = pid->kd*(pid->err[1]-pid->err[0]);	//最后一个是误差微分部分的输出
		
		VAL_LIMIT(pid->output_kp,-pid->output_kpMax,pid->output_kpMax);
		VAL_LIMIT(pid->output_ki,-pid->output_kiMax,pid->output_kiMax);//一般在这项做限制作为抗饱和积分//上下两个加不加以后看
		VAL_LIMIT(pid->output_kd,-pid->output_kdMax,pid->output_kdMax);
		pid->output = pid->output_kp + pid->output_ki + pid->output_kd;

		
	}
	else
	{//试试看阶梯式积分
		if(pid->err[1] > pid->ref/10)//感觉极其有道理
		{
			pid->err[0] = 0;
		}
//		else if(pid->err[1] > pid->ref/100)
//		{
//			pid->err[0] /= 2; 
//		}
		else
		{
			pid->index = (1-pid->err[1]/pid->ref);
			pid->err[0] *= pid->index;
		}
		if(pid->err[1]<0)
		{
			
			
			
			pid->err[1] = 0;//kp
		}
		
		//还可以对微分项加以改进。思路是1.不完全微分算法，在输出端加一个一阶低通滤波（会让反应变慢一点）2.对给定值微分，而不是偏差值e(t),这样就没有了对目标值变化的微分作用
		
		
		
		
		
		pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0]);
	
	}
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}
/**
	* @brief PID计算
	* @param PID_Regulator_t *PID_Stucture
	* @param float ref
	* @param float fdb
	* @retval float output
*/
float PID_Task(PID_Regulator_t *PID_Stucture, float ref, float fdb)
{
	PID_Stucture->ref = ref;
	PID_Stucture->fdb = fdb;
	PID_Stucture->Calc(PID_Stucture);
	return PID_Stucture->output;
}

/*
	模糊规则
	kp*
	e	|										ec
		|	NB		NM		NS		ZO		PS		PM		PB
		|
	NB| PB		PB		PM		PB		PS		ZO		ZO
	NM|	PB		PB		PM		PS		PS		ZO		NS
	NS|	PM		PM		PM		PS		ZO		NS		NS
	ZO|	PM		PM		PS		ZO		BS		NM		NM
	PS|	PS		OS		ZO		NS		NS		NM		NM
	PM|	PS		ZO		NS		NM		NM		NM		NB
	PB|	ZO		ZO		NM		NM		NM		NB		NB

	ki*
	e	|										ec
		|	NB		NM		NS		ZO		PS		PM		PB
		|
	NB| NB		NB		NM		NM		NS		ZO		ZO
	NM|	NB		NB		NM		NS		NS		ZO		NS
	NS|	NB		NM		NS		NS		ZO		PS		PS
	ZO|	NM		NM		NS		ZO		PS		PM		PM
	PS|	NM		NS		ZO		PS		PS		PB		PB
	PM|	ZO		ZO		PS		PS		PM		PB		PB
	PB|	ZO		ZO		PS		PM		PM		PB		PB
	
	kd*
	e	|										ec
		|	NB		NM		NS		ZO		PS		PM		PB
		|
	NB| PS		NS		NB		NB		NB		NM		PS
	NM|	PS		NS		NM		NM		NM		NS		ZO
	NS|	ZO		NS		NS		NM		NS		NS		ZO
	ZO|	ZO		NS		NS		NS		NS		NS		ZO
	PS|	ZO		ZO		ZO		ZO		ZO		ZO		ZO
	PM|	PB		NS		PS		PS		PS		PS		PB
	PB|	PB		PM		PM		PM		PS		PS		PB


https://blog.csdn.net/shuoyueqishilove/article/details/78236541


























class FuzzyPID
{
public:
    const static int N=7;
private:
    float target;  //系统的控制目标
    float actual;  //采样获得的实际值
    float e;       //误差
    float e_pre_1; //上一次的误差
    float e_pre_2; //上上次的误差
    float de;      //误差的变化率
    float emax;    //误差基本论域上限
    float demax;   //误差辩化率基本论域的上限
    float delta_Kp_max;   //delta_kp输出的上限
    float delta_Ki_max;   //delta_ki输出上限
    float delta_Kd_max;   //delta_kd输出上限
    float Ke;      //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
    float Kde;     //Kde=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
    float Ku_p;    //Ku_p=Kpmax/n,量化论域为[-3,-2,-1,0,1,2,3]
    float Ku_i;    //Ku_i=Kimax/n,量化论域为[-3,-2,-1,0,1,2,3]
    float Ku_d;    //Ku_d=Kdmax/n,量化论域为[-3,-2,-1,0,1,2,3]
    int Kp_rule_matrix[N][N];//Kp模糊规则矩阵
    int Ki_rule_matrix[N][N];//Ki模糊规则矩阵
    int Kd_rule_matrix[N][N];//Kd模糊规则矩阵
    string mf_t_e;       //e的隶属度函数类型
    string mf_t_de;      //de的隶属度函数类型
    string mf_t_Kp;      //kp的隶属度函数类型
    string mf_t_Ki;      //ki的隶属度函数类型
    string mf_t_Kd;      //kd的隶属度函数类型
    float *e_mf_paras; //误差的隶属度函数的参数
    float *de_mf_paras;//误差的偏差隶属度函数的参数
    float *Kp_mf_paras; //kp的隶属度函数的参数
    float *Ki_mf_paras; //ki的隶属度函数的参数
    float *Kd_mf_paras; //kd的隶属度函数的参数
    float Kp;
    float Ki;
    float Kd;
    float A;
    float B;
    float C;
void showMf(const string & type,float *mf_paras);      //显示隶属度函数的信息
void setMf_sub(const string & type,float *paras,int n);//设置模糊隶属度函数的子函数
public:
FuzzyPID(float e_max,float de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0);
FuzzyPID(float *fuzzyLimit,float *pidInitVal);
~FuzzyPID();
float trimf(float x,float a,float b,float c);          //三角隶属度函数
float gaussmf(float x,float ave,float sigma);          //正态隶属度函数
float trapmf(float x,float a,float b,float c,float d); //梯形隶属度函数
void setMf(const string & mf_type_e,float *e_mf,
               const string & mf_type_de,float *de_mf,
               const string & mf_type_Kp,float *Kp_mf,
               const string & mf_type_Ki,float *Ki_mf,
               const string & mf_type_Kd,float *Kd_mf); //设置模糊隶属度函数的参数
void setRuleMatrix(int kp_m[N][N],int ki_m[N][N],int kd_m[N][N]);  //设置模糊规则
float realize(float t,float a);  //实现模糊控制
void showInfo();   //显示该模糊控制器的信息
};
--------------------- 



#include"fuzzy_PID.h"


FuzzyPID::FuzzyPID(float e_max,float de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0):
target(0),actual(0),emax(e_max),demax(de_max),delta_Kp_max(kp_max),delta_Ki_max(ki_max),delta_Kd_max(kd_max),e_mf_paras(NULL),de_mf_paras(NULL),
Kp_mf_paras(NULL),Ki_mf_paras(NULL),Kd_mf_paras(NULL)
{
   e=target-actual;
   e_pre_1=0;
   e_pre_2=0;
   de=e-e_pre_1;
   Ke=(N/2)/emax;
   Kde=(N/2)/demax;
   Ku_p=delta_Kp_max/(N/2);
   Ku_i=delta_Ki_max/(N/2);
   Ku_d=delta_Kd_max/(N/2);
   mf_t_e="No type";
   mf_t_de="No type";
   mf_t_Kp="No type";
   mf_t_Ki="No type";
   mf_t_Kd="No type";
   Kp=Kp0;
   Ki=Ki0;
   Kd=Kd0;
   A=Kp+Ki+Kd;
   B=-2*Kd-Kp;
   C=Kd;
}

FuzzyPID::FuzzyPID(float *fuzzyLimit,float *pidInitVal)
{
    target=0;
    actual=0;
    e=0;
    e_pre_1=0;
    e_pre_2=0;
    de=e-e_pre_1;
    emax=fuzzyLimit[0];
    demax=fuzzyLimit[1];
    delta_Kp_max=fuzzyLimit[2];
    delta_Ki_max=fuzzyLimit[3];
    delta_Kd_max=fuzzyLimit[4];
    Ke=(N/2)/emax;
    Kde=(N/2)/demax;
    Ku_p=delta_Kp_max/(N/2);
    Ku_i=delta_Ki_max/(N/2);
    Ku_d=delta_Kd_max/(N/2);
    mf_t_e="No type";
    mf_t_de="No type";
    mf_t_Kp="No type";
    mf_t_Ki="No type";
    mf_t_Kd="No type";
    e_mf_paras=NULL;
    de_mf_paras=NULL;
    Kp_mf_paras=NULL;
    Ki_mf_paras=NULL;
    Kd_mf_paras=NULL;

    Kp=pidInitVal[0];
    Ki=pidInitVal[1];
    Kd=pidInitVal[2];
    A=Kp+Ki+Kd;
    B=-2*Kd-Kp;
    C=Kd;
}

FuzzyPID::~FuzzyPID()
{
  delete [] e_mf_paras;
  delete [] de_mf_paras;
  delete [] Kp_mf_paras;
  delete [] Ki_mf_paras;
  delete [] Kd_mf_paras;
}
//三角隶属度函数
float FuzzyPID::trimf(float x,float a,float b,float c)
{
   float u;
   if(x>=a&&x<=b)
       u=(x-a)/(b-a);
   else if(x>b&&x<=c)
       u=(c-x)/(c-b);
   else
       u=0.0;
   return u;

}
//正态隶属度函数
float FuzzyPID::gaussmf(float x,float ave,float sigma) 
{
    float u;
    if(sigma<0)
    {
       cout<<"In gaussmf, sigma must larger than 0"<<endl;
    }
    u=exp(-pow(((x-ave)/sigma),2));
    return u;
}
//梯形隶属度函数
float FuzzyPID::trapmf(float x,float a,float b,float c,float d)
{
    float u;
    if(x>=a&&x<b)
        u=(x-a)/(b-a);
    else if(x>=b&&x<c)
        u=1;
    else if(x>=c&&x<=d)
        u=(d-x)/(d-c);
    else
        u=0;
    return u;
}
//设置模糊规则Matrix
void FuzzyPID::setRuleMatrix(int kp_m[N][N],int ki_m[N][N],int kd_m[N][N])
{
    for(int i=0;i<N;i++)
       for(int j=0;j<N;j++)
       {
           Kp_rule_matrix[i][j]=kp_m[i][j];
           Ki_rule_matrix[i][j]=ki_m[i][j];
           Kd_rule_matrix[i][j]=kd_m[i][j];
       }
}
//设置模糊隶属度函数的子函数
void FuzzyPID::setMf_sub(const string & type,float *paras,int n)
{
    int N_mf_e,N_mf_de,N_mf_Kp,N_mf_Ki,N_mf_Kd;
  switch(n)
  {
  case 0:
      if(type=="trimf"||type=="gaussmf"||type=="trapmf")
        mf_t_e=type;
      else
        cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_e=="trimf")
        N_mf_e=3;
      else if(mf_t_e=="gaussmf")
        N_mf_e=2;
      else if(mf_t_e=="trapmf")
        N_mf_e=4;

      e_mf_paras=new float [N*N_mf_e];
      for(int i=0;i<N*N_mf_e;i++)
        e_mf_paras[i]=paras[i];
      break;

  case 1:
      if(type=="trimf"||type=="gaussmf"||type=="trapmf")
        mf_t_de=type;
      else
        cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_de=="trimf")
        N_mf_de=3;
      else if(mf_t_de=="gaussmf")
        N_mf_de=2;
      else if(mf_t_de=="trapmf")
        N_mf_de=4;
        de_mf_paras=new float [N*N_mf_de];
      for(int i=0;i<N*N_mf_de;i++)
        de_mf_paras[i]=paras[i];
      break;

   case 2:
      if(type=="trimf"||type=="gaussmf"||type=="trapmf")
        mf_t_Kp=type;
      else
        cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Kp=="trimf")
        N_mf_Kp=3;
      else if(mf_t_Kp=="gaussmf")
        N_mf_Kp=2;
      else if(mf_t_Kp=="trapmf")
        N_mf_Kp=4;
        Kp_mf_paras=new float [N*N_mf_Kp];
      for(int i=0;i<N*N_mf_Kp;i++)
        Kp_mf_paras[i]=paras[i];
      break;

   case 3:
      if(type=="trimf"||type=="gaussmf"||type=="trapmf")
        mf_t_Ki=type;
      else
        cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Ki=="trimf")
        N_mf_Ki=3;
      else if(mf_t_Ki=="gaussmf")
        N_mf_Ki=2;
      else if(mf_t_Ki=="trapmf")
        N_mf_Ki=4;
        Ki_mf_paras=new float [N*N_mf_Ki];
      for(int i=0;i<N*N_mf_Ki;i++)
        Ki_mf_paras[i]=paras[i];
      break;

   case 4:
      if(type=="trimf"||type=="gaussmf"||type=="trapmf")
        mf_t_Kd=type;
      else
        cout<<"Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\""<<endl;
      if(mf_t_Kd=="trimf")
        N_mf_Kd=3;
      else if(mf_t_Kd=="gaussmf")
        N_mf_Kd=2;
      else if(mf_t_Kd=="trapmf")
        N_mf_Kd=4;
        Kd_mf_paras=new float [N*N_mf_Kd];
      for(int i=0;i<N*N_mf_Kd;i++)
        Kd_mf_paras[i]=paras[i];
      break;

   default: break;
  }
}
//设置模糊隶属度函数的类型和参数
void FuzzyPID::setMf(const string & mf_type_e,float *e_mf,
            const string & mf_type_de,float *de_mf,
            const string & mf_type_Kp,float *Kp_mf,
            const string & mf_type_Ki,float *Ki_mf,
            const string & mf_type_Kd,float *Kd_mf)
{
    setMf_sub(mf_type_e,e_mf,0);
    setMf_sub(mf_type_de,de_mf,1);
    setMf_sub(mf_type_Kp,Kp_mf,2);
    setMf_sub(mf_type_Ki,Ki_mf,3);
    setMf_sub(mf_type_Kd,Kd_mf,4);
}
实现模糊控制
float FuzzyPID::realize(float t,float a)   
{
    float u_e[N],u_de[N],u_u[N];
    int u_e_index[3],u_de_index[3];//假设一个输入最多激活3个模糊子集
    float delta_Kp,delta_Ki,delta_Kd;
    float delta_u;
    target=t;
    actual=a;
    e=target-actual;
    de=e-e_pre_1;
    e=Ke*e;
    de=Kde*de;
  将误差e模糊化
    int j=0;
    for(int i=0;i<N;i++)
    {
        if(mf_t_e=="trimf")
          u_e[i]=trimf(e,e_mf_paras[i*3],e_mf_paras[i*3+1],e_mf_paras[i*3+2]);//e模糊化，计算它的隶属度
        else if(mf_t_e=="gaussmf")
          u_e[i]=gaussmf(e,e_mf_paras[i*2],e_mf_paras[i*2+1]);//e模糊化，计算它的隶属度
        else if(mf_t_e=="trapmf")
          u_e[i]=trapmf(e,e_mf_paras[i*4],e_mf_paras[i*4+1],e_mf_paras[i*4+2],e_mf_paras[i*4+3]);//e模糊化，计算它的隶属度

        if(u_e[i]!=0)
            u_e_index[j++]=i;                //存储被激活的模糊子集的下标，可以减小计算量
    }
    for(;j<3;j++)u_e_index[j]=0;             //富余的空间填0

    /*将误差变化率de模糊化
    j=0;
    for(int i=0;i<N;i++)
    {
        if(mf_t_de=="trimf")
           u_de[i]=trimf(de,de_mf_paras[i*3],de_mf_paras[i*3+1],de_mf_paras[i*3+2]);//de模糊化，计算它的隶属度
        else if(mf_t_de=="gaussmf")
           u_de[i]=gaussmf(de,de_mf_paras[i*2],de_mf_paras[i*2+1]);//de模糊化，计算它的隶属度
        else if(mf_t_de=="trapmf")
           u_de[i]=trapmf(de,de_mf_paras[i*4],de_mf_paras[i*4+1],de_mf_paras[i*4+2],de_mf_paras[i*4+3]);//de模糊化，计算它的隶属度

        if(u_de[i]!=0)
            u_de_index[j++]=i;            //存储被激活的模糊子集的下标，可以减小计算量
    }
    for(;j<3;j++)u_de_index[j]=0;          //富余的空间填0

    float den=0,num=0;
    /*计算delta_Kp和Kp
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
           num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Kp_rule_matrix[u_e_index[m]][u_de_index[n]];
           den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
        }
    delta_Kp=num/den;
    delta_Kp=Ku_p*delta_Kp;
    if(delta_Kp>=delta_Kp_max)   delta_Kp=delta_Kp_max;
    else if(delta_Kp<=-delta_Kp_max) delta_Kp=-delta_Kp_max;
    Kp+=delta_Kp;
    if(Kp<0)Kp=0;
    /*计算delta_Ki和Ki
    den=0;num=0;
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
           num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Ki_rule_matrix[u_e_index[m]][u_de_index[n]];
           den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
        }

    delta_Ki=num/den;
    delta_Ki=Ku_i*delta_Ki;
    if(delta_Ki>=delta_Ki_max)   delta_Ki=delta_Ki_max;
    else if(delta_Ki<=-delta_Ki_max)  delta_Ki=-delta_Ki_max;
    Ki+=delta_Ki;
    if(Ki<0)Ki=0;
    /*计算delta_Kd和Kd
    den=0;num=0;
    for(int m=0;m<3;m++)
        for(int n=0;n<3;n++)
        {
           num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*Kd_rule_matrix[u_e_index[m]][u_de_index[n]];
           den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
        }
    delta_Kd=num/den;
    delta_Kd=Ku_d*delta_Kd;
    if(delta_Kd>=delta_Kd_max)   delta_Kd=delta_Kd_max;
    else if(delta_Kd<=-delta_Kd_max) delta_Kd=-delta_Kd_max;
    Kd+=delta_Kd;
    if(Kd<0)Kd=0;

    A=Kp+Ki+Kd;
    B=-2*Kd-Kp;
    C=Kd;
    delta_u=A*e+B*e_pre_1+C*e_pre_2;

    delta_u=delta_u/Ke;

    if(delta_u>=0.95*target)delta_u=0.95*target;
    else if(delta_u<=-0.95*target)delta_u=-0.95*target;

    e_pre_2=e_pre_1;
    e_pre_1=e;

    return delta_u;
}
void FuzzyPID::showMf(const string & type,float *mf_paras)
{
    int tab;
    if(type=="trimf")
        tab=2;
    else if(type=="gaussmf")
        tab==1;
    else if(type=="trapmf")
        tab=3;
    cout<<"函数类型："<<mf_t_e<<endl;
    cout<<"函数参数列表："<<endl;
    float *p=mf_paras;
    for(int i=0;i<N*(tab+1);i++)
      {
          cout.width(3);
          cout<<p[i]<<"  ";
          if(i%(tab+1)==tab)
              cout<<endl;
      }
}
void FuzzyPID::showInfo()
{
   cout<<"Info of this fuzzy controller is as following:"<<endl;
   cout<<"基本论域e：["<<-emax<<","<<emax<<"]"<<endl;
   cout<<"基本论域de：["<<-demax<<","<<demax<<"]"<<endl;
   cout<<"基本论域delta_Kp：["<<-delta_Kp_max<<","<<delta_Kp_max<<"]"<<endl;
   cout<<"基本论域delta_Ki：["<<-delta_Ki_max<<","<<delta_Ki_max<<"]"<<endl;
   cout<<"基本论域delta_Kd：["<<-delta_Kd_max<<","<<delta_Kd_max<<"]"<<endl;
   cout<<"误差e的模糊隶属度函数参数："<<endl;
   showMf(mf_t_e,e_mf_paras);
   cout<<"误差变化率de的模糊隶属度函数参数："<<endl;
   showMf(mf_t_de,de_mf_paras);
   cout<<"delta_Kp的模糊隶属度函数参数："<<endl;
   showMf(mf_t_Kp,Kp_mf_paras);
   cout<<"delta_Ki的模糊隶属度函数参数："<<endl;
   showMf(mf_t_Ki,Ki_mf_paras);
   cout<<"delta_Kd的模糊隶属度函数参数："<<endl;
   showMf(mf_t_Kd,Kd_mf_paras);
   cout<<"模糊规则表："<<endl;
   cout<<"delta_Kp的模糊规则矩阵"<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<Kp_rule_matrix[i][j]<<"  ";
        }
       cout<<endl;
   }
   cout<<"delta_Ki的模糊规则矩阵"<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<Ki_rule_matrix[i][j]<<"  ";
        }
       cout<<endl;
   }
   cout<<"delta_Kd的模糊规则矩阵"<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<Kd_rule_matrix[i][j]<<"  ";
        }
       cout<<endl;
   }
   cout<<endl;
   cout<<"误差的量化比例因子Ke="<<Ke<<endl;
   cout<<"误差变化率的量化比例因子Kde="<<Kde<<endl;
   cout<<"输出的量化比例因子Ku_p="<<Ku_p<<endl;
   cout<<"输出的量化比例因子Ku_i="<<Ku_i<<endl;
   cout<<"输出的量化比例因子Ku_d="<<Ku_d<<endl;
   cout<<"设定目标target="<<target<<endl;
   cout<<"误差e="<<e<<endl;
   cout<<"Kp="<<Kp<<endl;
   cout<<"Ki="<<Ki<<endl;
   cout<<"Kd="<<Kd<<endl;
   cout<<endl;
}

之后是简单的测试：



#include<iostream>
#include"fuzzy_PID.h"

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

int main()
{
    float target=600;
    float actual=0;
    float u=0;
    int deltaKpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
                             {PB,PB,PM,PS,PS,ZO,NS},
                             {PM,PM,PM,PS,ZO,NS,NS},
                             {PM,PM,PS,ZO,NS,NM,NM},
                             {PS,PS,ZO,NS,NS,NM,NM},
                             {PS,ZO,NS,NM,NM,NM,NB},
                             {ZO,ZO,NM,NM,NM,NB,NB}};
    int deltaKiMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
                             {NB,NB,NM,NS,NS,ZO,ZO},
                             {NB,NM,NS,NS,ZO,PS,PS},
                             {NM,NM,NS,ZO,PS,PM,PM},
                             {NM,NS,ZO,PS,PS,PM,PB},
                             {ZO,ZO,PS,PS,PM,PB,PB},
                             {ZO,ZO,PS,PM,PM,PB,PB}};
    int deltaKdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},
                             {PS,NS,NB,NM,NM,NS,ZO},
                             {ZO,NS,NM,NM,NS,NS,ZO},
                             {ZO,NS,NS,NS,NS,NS,ZO},
                             {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                             {PB,NS,PS,PS,PS,PS,PB},
                             {PB,PM,PM,PM,PS,PS,PB}};
    float e_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    float de_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    float Kp_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    float Ki_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    float Kd_mf_paras[]={-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    FuzzyPID fuzzypid(1500,650,0.3,0.9,0.6,0.01,0.04,0.01);
    fuzzypid.setMf("trimf",e_mf_paras,"trimf",de_mf_paras,"trimf",Kp_mf_paras,"trimf",Ki_mf_paras,"trimf",Kd_mf_paras);
    fuzzypid.setRuleMatrix(deltaKpMatrix,deltaKiMatrix,deltaKdMatrix);
    cout<<"num target    actual"<<endl;
   // fuzzy.showInfo();
    for(int i=0;i<50;i++)
    {
        u=fuzzypid.realize(target,actual);
        actual+=u;
        cout<<i<<"   "<<target<<"    "<<actual<<endl;
    }
    fuzzypid.showInfo();
    system("pause");
    return 0;
}
--------------------- 
作者：shuoyueqishilove 
来源：CSDN 
原文：https://blog.csdn.net/shuoyueqishilove/article/details/78236541 
版权声明：本文为博主原创文章，转载请附上博文链接！




*/
