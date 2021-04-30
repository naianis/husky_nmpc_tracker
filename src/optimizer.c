/* **************************************************************************** */
/*                                 user functions                               */
/* **************************************************************************** */
#include "o8para.h"
#define  X extern
#include "o8comm.h"
#include "o8fint.h"
#include "o8cons.h"
#undef   X
#include "stdio.h"
#include "stdlib.h"
#include "matrix.h"



int Hp, Hc; 
double x_0[41];
int f1=0;
double Ts;
double Curv;
double sp;
double f=0;



int i = 0;
int ii = 0;

MAT   *Q;
MAT   *R;

MAT   *xb;
MAT   *xbcon;
MAT   *ub;
MAT   *ubcon;

void optimizer(double Params[], double x00[], double u00[],double result[]){  

double tetae, alphae, xe, ye = 0;
	
void donlp2(void);
void solchk(void);
void error_model(void);
void constraints(void);


//Definição das constantes 
////Horizonte de predição
Hp=Params[0];
//Horizonte de controle
Hc=Params[1]; 
//Período de amostragem
Ts=Params[4]; 
//Curvatura no trecho de caminho
Curv=Params[5];
//Taxa de progressão do veículo virtual 
sp=Params[6]; 

x_0[0] = 0;
for(i=1;i<=2*(Hc+1);i++){
	x_0[i] = u00[i-1];
}
i=0;


//Matriz de ponderação dos estados
Q=m_get(3,3);
m_ident(Q);
sm_mlt(Params[2],Q,Q);
//Q->me[2][2]=(Params[2]*0.01);
m_output(Q);

//Matriz de ponderação das entradas
R=m_get(2,2);
m_ident(R);
sm_mlt(Params[3],R,R);
//R->me[2][2]=(Params[3]*0.01);
//m_output(R);

//Matrizes de estados atuais e preditos
xb=m_get(3,Hp+1);
for(i=0;i<=Hp;i++){
xb->me[0][i]=x00[3*i];
xb->me[1][i]=x00[3*i+1];
xb->me[2][i]=x00[3*i+2];
}
//m_output(xb);


//Matrizes de estados atuais e preditos
xbcon=m_get(3,Hp+1);
for(i=0;i<=Hp;i++){
xbcon->me[0][i]=x00[3*i];
xbcon->me[1][i]=x00[3*i+1];
xbcon->me[2][i]=x00[3*i+2];
}
//m_output(xbcon);

//Matrizes de entradas atuais
ub=m_get(2,Hc+1);
for(i=0;i<=Hc;i++){
ub->me[0][i]=u00[2*i];
ub->me[1][i]=u00[2*i+1];
//ub->me[2][i]=u00[3*i+2];
}
//m_output(ub);

//Matrizes de entradas atuais
ubcon=m_get(2,Hc+1);
for(i=0;i<=Hc;i++){
ubcon->me[0][i]=u00[2*i];
ubcon->me[1][i]=u00[2*i+1];
//ubcon->me[2][i]=u00[3*i+2];
}
//m_output(ub);

i=0;


//chamada função otimizador
donlp2();

f=0;
result[0] = fx;
result[1] = x[1];
result[2] = x[2];
//result[3] = x[3];
//result[4] = x[4];


//alphae=xb->me[2][0]+Ts*x[2];
//tetae=xb->me[3][0]+Ts*x[3];

//sp=(0.5)*cos(alphae)-x[1];

//xe=xb->me[0][0]+Ts*((xb->me[1][0]*Curv*sp)-(sp)+(0.5*cos(alphae)));
//ye=xb->me[1][0]+Ts*(-xb->me[0][0]*Curv*sp+0.5*sin(alphae));

//printf("\nalphae=%e\n",alphae);
//printf("\ntetae=%e\n",tetae);
//printf("\nxe=%e\n",xe);
//printf("\nye=%e\n",ye);
//printf("\nTs=%e\n",Ts);


//M_FREE(Q);
//M_FREE(R);
//M_FREE(ub);
//M_FREE(xb);
//M_FREE(ubcon);
//M_FREE(xbcon);

return;
}


void error_model(){
int kk=0;
double u1,u2,u3=0;
double xe0,ye0,alphae0,thetae0,eta0=0;
double xe1,ye1,alphae1,thetae1,eta1=0;


for (kk=0;kk<=Hp;kk++){
	
u1 = ub->me[0][kk];
u2 = ub->me[1][kk];
//u3 = ub->me[2][kk];

xe0 = xb->me[0][kk];
ye0 = xb->me[1][kk];
thetae0 = xb->me[2][kk];

//trazer vnav e w!
thetae1=asin(u2/(0.07));
//printf("\nalphae_u2=%e\n",alphae1);

//thetae1=thetae0+Ts*(0.5-Curv*sp);

sp=(0.07)*cos(thetae1)-u1;

xe1=xe0+Ts*(ye0*Curv*sp+u1);
ye1=ye0+Ts*(-xe0*Curv*sp+u2);
//thetae1=thetae0+Ts*(0.5-Curv*sp);

xbcon->me[0][kk] = xe1;
xbcon->me[1][kk] = ye1;
xbcon->me[2][kk] = thetae1;

//printf("\nxe=%e\n",xe1);
printf("\nye=%e\n",ye1);
//printf("\nalphae=%e\n",alphae1);
//printf("\nthetae=%e\n",thetae1);

}	
	
return;	
	
}

/* **************************************************************************** */
/*                              donlp2-intv size initialization                 */
/* **************************************************************************** */
void user_init_size(void){
    n      = 2*(Hc+1);
    nlin   =  0;
    nonlin =  3*(Hc+1);
    //nonlin =  0;
    iterma = 4000;
    nstep = 20;
}

/* **************************************************************************** */
/*                              donlp2-intv standard setup                           */
/* **************************************************************************** */


//FALTANDO ADICIONAR RESTRIÇÕES !!!


void user_init(void) {
    //static INTEGER  i,j;
    //static double   xst0[4];
                                  
    /* name is ident of the example/user and can be set at users will       */
    /* the first static character must be alphabetic. 40 characters maximum */

    strcpy(name,"teste1");
   
    
    /* x is initial guess and also holds the current solution */
    /* problem dimension n = dim(x), nlin=number of linear constraints
       nonlin = number of nonlinear constraints  */
    
    analyt = FALSE;
    //epsdif = 1.e-8;   /* gradients exact to machine precision */
    /* if you want numerical differentiation being done by donlp2 then:*/
    //epsfcn   = 1.e-8; /* function values exact to machine precision */
    epsdif = 1.e-14;   /* gradients exact to machine precision */
    /* if you want numerical differentiation being done by donlp2 then:*/
    epsfcn   = 1.e-16; /* function values exact to machine precision */

    /*  bloc    = TRUE; */
    /* if one wants to evaluate all functions  in an independent process */
    /* difftype = 3; *//* the most accurate and most expensive choice */
    difftype = 1;    
    bloc = FALSE;
    nreset = n;
    
    
    del0 = 0.2e0;
    tau0 = 1.e0;
    tau  = 0.1e0;
    
    
    
    for (i = 0 ; i <= n ; i++) {
       x[i] = x_0[i]; 
    }
    i=0;
    
      // printf("\n\n o valor de x[3] é: %e \n\n", x[3]);  
    

    /*  set lower and upper bounds */
    big = 1.e20;
    

//Restrições nas entradas
for (i=0;i<=Hp;i++){
low[2*i+1]=-0.07;
low[2*i+2]=-0.07;
//low[3*i+3]=-12;
up[2*i+1]=0.07;
up[2*i+2]=0.07;
//up[3*i+3]=12;
}
i=0;

//Restrições na saída ?? ANALISAR ??
for (i=0;i<=Hp;i++){
low[2*(Hp+1)+3*i+1]=-0.05;
low[2*(Hp+1)+3*i+2]=-0.05;
low[2*(Hp+1)+3*i+3]=-0.1;
up[2*(Hp+1)+3*i+1]=0.05;
up[2*(Hp+1)+3*i+2]=0.05;
up[2*(Hp+1)+3*i+3]=0.1;
}
i=0;
      
    return;
}

/* **************************************************************************** */
/*                                 special setup                                */
/* **************************************************************************** */
void setup(void) {
	//silent=TRUE; //RESOLVER PROBLEMA DE TRAVAMENTO QUANTO TRUE !!!!
    //intakt = TRUE;
    te0=FALSE;
    te1=FALSE;
    te2=FALSE;
    te3=FALSE;
    cold=FALSE;

    return;
}

/* **************************************************************************** */
/*  the user may add additional computations using the computed solution here   */
/* **************************************************************************** */
void solchk(void) {
    return;
}


/* **************************************************************************** */
/*                               objective function                             */
/* **************************************************************************** */
void ef(double x[],double *fx) {
//variáveis auxiliares
int k;
double f0;
VEC *xb0;
VEC *ub0;
VEC *xb0TQ;
VEC *ub0TR;


xb0=v_get(3);
ub0=v_get(2);
xb0TQ=v_get(3);
ub0TR=v_get(2);	
	
for (k=0;k<=Hp;k++){
ub->me[0][k]=x[2*k+1];
ub->me[1][k]=x[2*k+2];
//ub->me[2][k]=x[3*k+3];
}
//m_output(ub);
k=0;

error_model();

f=0;
for (k=0;k<=Hp;k++){
//printf("\nk=%d\n",k);

	get_col(xbcon,k,xb0);
	get_col(ub,k,ub0);
	
	
	
	xb0TQ = vm_mlt(Q,xb0,xb0TQ);
	ub0TR = vm_mlt(R,ub0,ub0TR);
	

	f0 = in_prod(xb0TQ,xb0) + in_prod(ub0TR,ub0);
	
	
	//printf("\nf0=%e\n",f0);
	f = f + Ts*f0;
	//printf("\nf=%e\n",f);
}
k=0;
*fx=f;
//getchar();

//printf("\nfx=%e\n",*fx);

//printf("\nxe=%e\n",xbcon->me[0][0]);
//printf("\nye=%e\n",xbcon->me[1][0]);
//printf("\nalphae=%e\n",xbcon->me[2][0]);
//printf("\nthetae=%e\n",xbcon->me[3][0]);


v_free(xb0);
v_free(ub0);
v_free(xb0TQ);
v_free(ub0TR);
//getchar();
return;
}

/* **************************************************************************** */
/*                          gradient of objective function                      */
/* **************************************************************************** */
void egradf(DOUBLE x[],DOUBLE gradf[]) {
    return;
}

/* **************************************************************************** */
/*  compute nonlinear constraints */
/* **************************************************************************** */
void econ(INTEGER type ,INTEGER liste[], DOUBLE x[],DOUBLE con[],
             LOGICAL err[]) {

int k;
	
for (k=0;k<=Hp;k++){
ub->me[0][k]=x[2*k+1];
ub->me[1][k]=x[2*k+2];
//ub->me[2][k]=x[3*k+3];
}	

error_model();
			 
//constraints();

for (k=0;k<=Hp;k++){
//printf("\nk=%d\n",k);
con[3*k+1]=xbcon->me[0][k];
con[3*k+2]=xbcon->me[1][k];
con[3*k+3]=xbcon->me[2][k];
//err[2]=TRUE;
}

//printf("\nxe=%e\n",con[1]);
//printf("\nye=%e\n",con[2]);
//printf("\nalphae=%e\n",con[3]);
//printf("\nthetae=%e\n",con[4]);


return;

}

/* **************************************************************************** */
/*          compute the gradient of the  nonlinear constraints               */
/* **************************************************************************** */
void econgrad(INTEGER liste[] ,INTEGER shift ,  DOUBLE x[], DOUBLE **grad) {
	return;
}

/* **************************************************************************** */
/*                        user functions (if bloc == TRUE)                      */
/* **************************************************************************** */
void eval_extern(INTEGER mode) {

return;

}
