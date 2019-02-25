
/* Includes ------------------------------------------------------------------*/

#include "PID.h"




PID_Regulator_t RBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t LBMSpeedPID = BELT_MOTOR_SPEED_PID_DEFAULT;//λ�û�Ҫ��Ҫ���Ժ���˵
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
	pid->err[0]=0;//err0��Ϊ������
	pid->err[1]=0;//err1��Ϊ�������
	pid->err[2]=0;//err2��Ϊ�ϴ����
	pid->err[3]=0;//err2��Ϊ���ϴ����
	pid->output=0;
}





void PID_Calc(PID_Regulator_t *pid)
{
	
	pid->err[3]=pid->err[2];//ֻ��������ʽpid�в��õ������ϴ���err3
	pid->err[2]=pid->err[1];//�ϴ����
	pid->err[1]=pid->ref-pid->fdb;//�������
	pid->err[0]+=pid->err[1];//������
	//VAL_LIMIT(pid->ki,-pid->componentKiMax,pid->componentKiMax);	//�����ͻ���//���Ǵ�ģ�����������output�ı��������ki�ˣ�ɾ����
	if(pid->type == POSITION_PID)
	{
		pid->output_kp = pid->kp*pid->err[1];//��������
		pid->output_ki = pid->ki*pid->err[2];//���ֲ���
		pid->output_kd = pid->kd*(pid->err[1]-pid->err[0]);	//���һ�������΢�ֲ��ֵ����
		
		VAL_LIMIT(pid->output_kp,-pid->output_kpMax,pid->output_kpMax);
		VAL_LIMIT(pid->output_ki,-pid->output_kiMax,pid->output_kiMax);//һ����������������Ϊ�����ͻ���//���������Ӳ����Ժ�
		VAL_LIMIT(pid->output_kd,-pid->output_kdMax,pid->output_kdMax);
		pid->output = pid->output_kp + pid->output_ki + pid->output_kd;
	}
	else if(pid->type == DELTA_PID)
	{
		pid->output = pid->kp*(pid->err[1]-pid->err[2])+pid->ki*pid->err[1]+pid->kd*(pid->err[1] - 2*pid->err[2]+pid->err[3]);//����ʽpid��ֻʹ�����м��书�ܵ��������粽�������
	}
	else if(pid->type == VAGUE_PID)
	{

		static float kp_delta = 0;
		static float ki_delta = 0;
		static float kd_delta = 0;
		
		EC = (pid->err[2]-pid->err[1]);//���仯��,�����ĵ�λ�����/ms
		E = 	pid->err[1];
		

								//*6��Ϊ�˻���0-6������
//		E_Index = E*6/pid->refmax - (E*6/pid->refmax - 1);//����������,��һ����1-E_Index
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

		
		
		
		
		//���int8_t ǿ��ת���ɱ�������Ǹ�����
//		ki_delta = pid->ki * deltaKiMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)];
//		kd_delta = pid->kd * deltaKdMatrix[(int8_t)(E*6/pid->refmax+3)][(int8_t)(EC*6/pid->ecmax+3)];
		
		
		
		
		pid->kp = pid->kp+kp_delta;
		pid->ki = pid->ki+ki_delta;
		pid->kd = pid->kd+kd_delta;

		pid->output_kp = pid->kp*pid->err[1];//��������
		pid->output_ki = pid->ki*pid->err[2];//���ֲ���
		pid->output_kd = pid->kd*(pid->err[1]-pid->err[0]);	//���һ�������΢�ֲ��ֵ����
		
		VAL_LIMIT(pid->output_kp,-pid->output_kpMax,pid->output_kpMax);
		VAL_LIMIT(pid->output_ki,-pid->output_kiMax,pid->output_kiMax);//һ����������������Ϊ�����ͻ���//���������Ӳ����Ժ�
		VAL_LIMIT(pid->output_kd,-pid->output_kdMax,pid->output_kdMax);
		pid->output = pid->output_kp + pid->output_ki + pid->output_kd;

		
	}
	else
	{//���Կ�����ʽ����
		if(pid->err[1] > pid->ref/10)//�о������е���
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
		
		//�����Զ�΢������ԸĽ���˼·��1.����ȫ΢���㷨��������˼�һ��һ�׵�ͨ�˲������÷�Ӧ����һ�㣩2.�Ը���ֵ΢�֣�������ƫ��ֵe(t),������û���˶�Ŀ��ֵ�仯��΢������
		
		
		
		
		
		pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0]);
	
	}
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}
/**
	* @brief PID����
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
	ģ������
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
    float target;  //ϵͳ�Ŀ���Ŀ��
    float actual;  //������õ�ʵ��ֵ
    float e;       //���
    float e_pre_1; //��һ�ε����
    float e_pre_2; //���ϴε����
    float de;      //���ı仯��
    float emax;    //��������������
    float demax;   //���绯�ʻ������������
    float delta_Kp_max;   //delta_kp���������
    float delta_Ki_max;   //delta_ki�������
    float delta_Kd_max;   //delta_kd�������
    float Ke;      //Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Kde;     //Kde=n/demax,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Ku_p;    //Ku_p=Kpmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Ku_i;    //Ku_i=Kimax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
    float Ku_d;    //Ku_d=Kdmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
    int Kp_rule_matrix[N][N];//Kpģ���������
    int Ki_rule_matrix[N][N];//Kiģ���������
    int Kd_rule_matrix[N][N];//Kdģ���������
    string mf_t_e;       //e�������Ⱥ�������
    string mf_t_de;      //de�������Ⱥ�������
    string mf_t_Kp;      //kp�������Ⱥ�������
    string mf_t_Ki;      //ki�������Ⱥ�������
    string mf_t_Kd;      //kd�������Ⱥ�������
    float *e_mf_paras; //���������Ⱥ����Ĳ���
    float *de_mf_paras;//����ƫ�������Ⱥ����Ĳ���
    float *Kp_mf_paras; //kp�������Ⱥ����Ĳ���
    float *Ki_mf_paras; //ki�������Ⱥ����Ĳ���
    float *Kd_mf_paras; //kd�������Ⱥ����Ĳ���
    float Kp;
    float Ki;
    float Kd;
    float A;
    float B;
    float C;
void showMf(const string & type,float *mf_paras);      //��ʾ�����Ⱥ�������Ϣ
void setMf_sub(const string & type,float *paras,int n);//����ģ�������Ⱥ������Ӻ���
public:
FuzzyPID(float e_max,float de_max,float kp_max,float ki_max,float kd_max,float Kp0,float Ki0,float Kd0);
FuzzyPID(float *fuzzyLimit,float *pidInitVal);
~FuzzyPID();
float trimf(float x,float a,float b,float c);          //���������Ⱥ���
float gaussmf(float x,float ave,float sigma);          //��̬�����Ⱥ���
float trapmf(float x,float a,float b,float c,float d); //���������Ⱥ���
void setMf(const string & mf_type_e,float *e_mf,
               const string & mf_type_de,float *de_mf,
               const string & mf_type_Kp,float *Kp_mf,
               const string & mf_type_Ki,float *Ki_mf,
               const string & mf_type_Kd,float *Kd_mf); //����ģ�������Ⱥ����Ĳ���
void setRuleMatrix(int kp_m[N][N],int ki_m[N][N],int kd_m[N][N]);  //����ģ������
float realize(float t,float a);  //ʵ��ģ������
void showInfo();   //��ʾ��ģ������������Ϣ
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
//���������Ⱥ���
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
//��̬�����Ⱥ���
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
//���������Ⱥ���
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
//����ģ������Matrix
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
//����ģ�������Ⱥ������Ӻ���
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
//����ģ�������Ⱥ��������ͺͲ���
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
ʵ��ģ������
float FuzzyPID::realize(float t,float a)   
{
    float u_e[N],u_de[N],u_u[N];
    int u_e_index[3],u_de_index[3];//����һ��������༤��3��ģ���Ӽ�
    float delta_Kp,delta_Ki,delta_Kd;
    float delta_u;
    target=t;
    actual=a;
    e=target-actual;
    de=e-e_pre_1;
    e=Ke*e;
    de=Kde*de;
  �����eģ����
    int j=0;
    for(int i=0;i<N;i++)
    {
        if(mf_t_e=="trimf")
          u_e[i]=trimf(e,e_mf_paras[i*3],e_mf_paras[i*3+1],e_mf_paras[i*3+2]);//eģ��������������������
        else if(mf_t_e=="gaussmf")
          u_e[i]=gaussmf(e,e_mf_paras[i*2],e_mf_paras[i*2+1]);//eģ��������������������
        else if(mf_t_e=="trapmf")
          u_e[i]=trapmf(e,e_mf_paras[i*4],e_mf_paras[i*4+1],e_mf_paras[i*4+2],e_mf_paras[i*4+3]);//eģ��������������������

        if(u_e[i]!=0)
            u_e_index[j++]=i;                //�洢�������ģ���Ӽ����±꣬���Լ�С������
    }
    for(;j<3;j++)u_e_index[j]=0;             //����Ŀռ���0

    /*�����仯��deģ����
    j=0;
    for(int i=0;i<N;i++)
    {
        if(mf_t_de=="trimf")
           u_de[i]=trimf(de,de_mf_paras[i*3],de_mf_paras[i*3+1],de_mf_paras[i*3+2]);//deģ��������������������
        else if(mf_t_de=="gaussmf")
           u_de[i]=gaussmf(de,de_mf_paras[i*2],de_mf_paras[i*2+1]);//deģ��������������������
        else if(mf_t_de=="trapmf")
           u_de[i]=trapmf(de,de_mf_paras[i*4],de_mf_paras[i*4+1],de_mf_paras[i*4+2],de_mf_paras[i*4+3]);//deģ��������������������

        if(u_de[i]!=0)
            u_de_index[j++]=i;            //�洢�������ģ���Ӽ����±꣬���Լ�С������
    }
    for(;j<3;j++)u_de_index[j]=0;          //����Ŀռ���0

    float den=0,num=0;
    /*����delta_Kp��Kp
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
    /*����delta_Ki��Ki
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
    /*����delta_Kd��Kd
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
    cout<<"�������ͣ�"<<mf_t_e<<endl;
    cout<<"���������б�"<<endl;
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
   cout<<"��������e��["<<-emax<<","<<emax<<"]"<<endl;
   cout<<"��������de��["<<-demax<<","<<demax<<"]"<<endl;
   cout<<"��������delta_Kp��["<<-delta_Kp_max<<","<<delta_Kp_max<<"]"<<endl;
   cout<<"��������delta_Ki��["<<-delta_Ki_max<<","<<delta_Ki_max<<"]"<<endl;
   cout<<"��������delta_Kd��["<<-delta_Kd_max<<","<<delta_Kd_max<<"]"<<endl;
   cout<<"���e��ģ�������Ⱥ���������"<<endl;
   showMf(mf_t_e,e_mf_paras);
   cout<<"���仯��de��ģ�������Ⱥ���������"<<endl;
   showMf(mf_t_de,de_mf_paras);
   cout<<"delta_Kp��ģ�������Ⱥ���������"<<endl;
   showMf(mf_t_Kp,Kp_mf_paras);
   cout<<"delta_Ki��ģ�������Ⱥ���������"<<endl;
   showMf(mf_t_Ki,Ki_mf_paras);
   cout<<"delta_Kd��ģ�������Ⱥ���������"<<endl;
   showMf(mf_t_Kd,Kd_mf_paras);
   cout<<"ģ�������"<<endl;
   cout<<"delta_Kp��ģ���������"<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<Kp_rule_matrix[i][j]<<"  ";
        }
       cout<<endl;
   }
   cout<<"delta_Ki��ģ���������"<<endl;
   for(int i=0;i<N;i++)
   {
     for(int j=0;j<N;j++)
       {
         cout.width(3);
         cout<<Ki_rule_matrix[i][j]<<"  ";
        }
       cout<<endl;
   }
   cout<<"delta_Kd��ģ���������"<<endl;
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
   cout<<"����������������Ke="<<Ke<<endl;
   cout<<"���仯�ʵ�������������Kde="<<Kde<<endl;
   cout<<"�����������������Ku_p="<<Ku_p<<endl;
   cout<<"�����������������Ku_i="<<Ku_i<<endl;
   cout<<"�����������������Ku_d="<<Ku_d<<endl;
   cout<<"�趨Ŀ��target="<<target<<endl;
   cout<<"���e="<<e<<endl;
   cout<<"Kp="<<Kp<<endl;
   cout<<"Ki="<<Ki<<endl;
   cout<<"Kd="<<Kd<<endl;
   cout<<endl;
}

֮���Ǽ򵥵Ĳ��ԣ�



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
���ߣ�shuoyueqishilove 
��Դ��CSDN 
ԭ�ģ�https://blog.csdn.net/shuoyueqishilove/article/details/78236541 
��Ȩ����������Ϊ����ԭ�����£�ת���븽�ϲ������ӣ�




*/
