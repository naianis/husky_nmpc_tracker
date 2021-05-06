#!/usr/bin/env python
# encoding: utf-8
#--------------------------------------------------
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from opencv101.msg import *
from geometry_msgs.msg import Twist
from husky_nmpc_tracker.msg import SysStatus1, DeviationParams


class Robot():
  # parâmetros de classe, iguais para todos os robôs
  # parametro de seleção do controle visual
  VFCMode = 0
  # parâmetros de formação
  qsip = 0
  formationK1 = 0
  formationK3 = 0
  formationQ = 0
  formationP31 = 0
  nmpcVarsWs = 0
  nmpcVarsWu = 0
  nmpcVarsP = 0
  nmpcVarsAlpha = 0
  nmpcVarsBeta = 0


class TPath():
	def __init__(self):
		self.xe,self.ye,self.alphae = [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
		self.thetae = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.u1,self.u2,self.u3 = [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
		self.u10,self.u20,self.u30 = [0.0], [0.0], [0.0]
		self.vnav = 0.2
		self.w = 0.2
		self.thetap = 0.0
		self.theta = [0.0, 0.0]
		self.c = 0.0 #????????????? curvatura
		self.s = 0.2 #????????????? quanto ele já está andando
		self.alphae0 = 0.0
		self.alpha = 0.0
		self.sp = 0.0
		self.phi = 0.0
		self.ID = 0
		self.start = 0
		self.i = 0

class TNmpc():
	def __init__(self):
		self.Hc = 3		#horizonte de controle?
		self.Hp = 3		#Horizonte de predição?
		self.R = 0.01
		self.Q = 0.1
		self.u1,u2,u3 = 0.0, 0.0, 0.0
		self.xemin,xemax,yemin,yemax,thetaemin,thetaemax = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
		self.Ts = 0.2
		self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.u = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.Parameters = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.Result = [0.0, 0.0, 0.0]

class robot_state():
	def __init__(self):
		self.xe = 0.0
		self.ye = 0.0
		self.tetae = 0.0
		self.x = 0.0
		self.y = 0.0
		self.teta = 0.0
		self.v = 0.0
		self.w = 0.0


class PathFollowing(object):


	def __init__(self):
		self.sub_obj = rospy.Subscriber('/odometry/filtered', Odometry, self.OdomCb)
		rospy.Subscriber("/resultadosOtimizador", optimizerResult, self.optimizerCb)
		rospy.Subscriber('/deviation_params', DeviationParams, self.deviationCb)

		self.optimizerPub = rospy.Publisher("/optimizerPub", optimizerParams, queue_size = 10)
		self.cmdVelPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)


		self.robot_pose = [0.0, 0.0, 0.0]

		#variaveis "adicionais" que criei -----------------------------
		self.RobotState = robot_state() #POSICAO ROBO ------------------------------------------------
		self.RobotReference = robot_state()
		self.Robot = Robot()

		self.RobotState.x = 0.0

		#self.FPathFollowing = TPathFollowing()
		self.PathError = TPath()
		self.NmpcVars = TNmpc()
		self.arq = ""
		self.u_opt = [0.0, 0.0, 0.0]
		self.J_min = 0.0
		self.iDt = 0
		self.fopt = 1
		self.AngAdj = 0.0
		self.f11 = 0
		self.f12 = 0
		self.f21 = 0
		self.f22 = 0
		self.tol_x = 0.05
		self.tol_y = 0.05
		self.tol_ecl = 0.0707
		self.FlagSwithReference = False
		self.FlagResetS = True
		self.FlagThetar = True
		self.xr_temp, self.yr_temp, self.thetar_temp = 0.0, 0.0, 0.0
		self.Ts = 0.2

		self.Ti = 0.0
		self.Tf = 0.0
		self.z = 0.0

		self.useVisualControl = False



	#precisa realmente ter essas condições?
	# RobotReference seria apenas o próximo ponto?
	def TPathFollowing_GetReference(self):
		r,A = 1.0, 0.0 #R É O RAIO!!!!!!!!!!
		alpha_temp = 0.0
		k1 = 0.1 #theta gain to curve landing
		k2 = 0.5 #w gain to curve landing


		if (self.PathError.ID==0) and (not self.FlagSwithReference):

			#quem pe PathError.s??
			#quem é r?
			self.RobotReference.x = r * math.cos(self.PathError.s)
			self.RobotReference.y = r * math.sin(self.PathError.s)
			AngAdj = math.atan2(self.RobotReference.y, self.RobotReference.x)

			self.RobotReference.teta=math.pi/2+AngAdj

			self.PathError.thetap = self.RobotReference.teta

			self.PathError.c =1/r #for the time being...contant!

			self.PathError.w = self.PathError.vnav/r

			xr_temp =self.RobotReference.x
			yr_temp =self.RobotReference.y
			thetar_temp =self.PathError.thetap

		xr_temp = self.RobotReference.x
		yr_temp = self.RobotReference.y
		thetar_temp = self.PathError.thetap

		self.TPathFollowing_CalcError(xr_temp,yr_temp,thetar_temp)

	def TPathFollowing_CalcError(self, x , y, theta):
		#calculo dos erros iniciais com relacao ao sistema de coordenadas do caminho
		self.PathError.xe[0] = math.cos(theta)*(self.RobotState.x - x)+math.sin(theta)*(self.RobotState.y-y)
		self.PathError.ye[0] = math.cos(theta)*(self.RobotState.y-y)-math.sin(theta)*(self.RobotState.x-x)
		#PathError.alphae[0] = PathError.alpha-PathError.thetap
		self.PathError.thetae[0]=self.RobotState.teta-theta

		self.RobotState.xe=self.PathError.xe[0]
		self.RobotState.ye =self.PathError.ye[0]
		self.RobotState.tetae =self.PathError.thetae[0]

		self.PathError.u1[0]=self.PathError.vnav*math.cos(self.PathError.thetae[0])-self.PathError.sp
		#self.PathError.u2[0]=self.PathError.vnav*math.sin(self.PathError.thetae[0])
		self.PathError.u3[0] = self.RobotReference.w-self.PathError.w

		print("erro xe ") +str(self.PathError.xe) + ("erro ye") +str(self.PathError.ye)

	def TPathFollowing_Predictor(self):
		i=0

		for i in range(int(self.NmpcVars.Hp)-1):
			self.PathError.xe[i+1] = self.PathError.xe[i]+self.Ts*(self.PathError.ye[i]*self.PathError.c*self.PathError.sp+self.PathError.u1[i])
			#self.PathError.ye[i+1] = self.PathError.ye[i]+self.Ts*(-self.PathError.xe[i]*self.PathError.c*self.PathError.sp+self.PathError.u2[i])
			self.PathError.ye[i+1] = self.PathError.ye[i]+self.Ts*(-self.PathError.xe[i]*self.PathError.c*self.PathError.sp+self.PathError.u3[i])
            #//self.PathError.alphae[i+1] = self.PathError.alphae[i]; //Check the possibilty of add alphaep
			self.PathError.thetae[i+1] = self.PathError.thetae[i]+self.NmpcVars.Ts*(self.RobotState.w-self.PathError.sp*self.PathError.c)

			self.PathError.u1[i+1] = self.PathError.vnav*math.cos(self.PathError.thetae[i+1])-self.PathError.sp
			self.PathError.u2[i+1] = self.PathError.vnav*math.sin(self.PathError.thetae[i+1])
			self.PathError.u3[i+1] = self.RobotReference.w-self.PathError.w
			print("predictor")

	def TPathFollowing_CallOptimizer(self):
		i = 0
		Hp = int(self.NmpcVars.Hp)
		Hc = int(self.NmpcVars.Hc)

		#//Parameters
		self.NmpcVars.Parameters[0] = self.NmpcVars.Hp
		self.NmpcVars.Parameters[1] = self.NmpcVars.Hc
		self.NmpcVars.Parameters[2] = self.NmpcVars.Q
		self.NmpcVars.Parameters[3] = self.NmpcVars.R
		self.NmpcVars.Parameters[4] = self.NmpcVars.Ts
		self.NmpcVars.Parameters[5] = self.PathError.c
		self.NmpcVars.Parameters[6] = self.PathError.sp


		for i in range(Hp):
			self.NmpcVars.x[3*i] = self.PathError.xe[i]
			self.NmpcVars.x[3*i+1] = self.PathError.ye[i]
			self.NmpcVars.x[3*i+2] = self.PathError.thetae[i]
			self.NmpcVars.u[2*i] = self.PathError.u1[i]
			self.NmpcVars.u[2*i+1] = self.PathError.u2[i]
			self.NmpcVars.u[3*i+2] = self.PathError.u3[i]



		self.vRef = 0.2
		self.wRef = 0.2
		self.horizontePredicao = 3
		self.horizonteControle = 3
		self.horizonte = 1.7
		self.Ts = 0.2
		self.s = 0.0
		self.sp = 0.0
		self.length = 0
		self.k = 0

		self.Q = 0.1
		self.R = 0.01
		self.id = 0
		#self.useVisualControl = False #TROCAR QUANDO NAO VISUAL -------------------------------
		# vetor de saída do otimizador
		self.otimizerResult = [0, 0, 0]


		# espacamento entre o lider e o self
		self.LFFClij = [0, 0, 0]
		self.LFFCpsiij = [0, 0, 0]
		self.LFFCbetaij = [0, 0, 0]
		self.LFFCgammaij = [0, 0, 0]

		# variáveis não mais utilizadas, mas que precisa por causa do otimizador
		self.xe = 0
		# valor da saída do otimizador um instante atrás
		self.last_opt = 0
		self.errorModelDep = 0

		# u é a ação de controle
		# de o desvio lateral previsto do caminho
		# thetae é o desvio angular previsto do caminho
		# cadê o u[0], é o u_opt[0]?
		self.x = [0, 0, 0, 0, 0, 0, 0]
		self.u = [0, 0, 0, 0, 0, 0, 0]
		self.de = [0, 0, 0]
		self.thetae = [0, 0, 0]

		parameters = [self.horizontePredicao, self.horizonteControle, self.Q, self.R, self.Ts, 0.0, Robot.qsip, self.id, self.vRef, self.v, self.wRef, 0, 0, 0,0, Robot.nmpcVarsWs, Robot.nmpcVarsWu, self.wRef, self.horizonte, self.horizonte, self.useVisualControl, self.Robot.nmpcVarsP, self.Robot.nmpcVarsAlpha, self.Robot.VFCMode, self.xe, self.last_opt, self.thetae[0], Robot.nmpcVarsBeta, self.errorModelDep, Robot.nmpcVarsBeta, self.v, self.s]

		h = std_msgs.msg.Header()
		h.stamp = rospy.Time.now()

		self.optimizerPub.publish(h, parameters, self.NmpcVars.x, self.NmpcVars.u, self.LFFClij, self.LFFCpsiij, self.LFFCbetaij, self.LFFCgammaij, self.otimizerResult);

		#self.optimizerPub.publish(h, parameters, self.x, self.u, self.LFFClij, self.LFFCpsiij, self.LFFCbetaij, self.LFFCgammaij, self.otimizerResult);

		#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		#optimizer(self.NmpcVars.Parameters,NmpcVars.x,NmpcVars.u,NmpcVars.Result)
		#NmpcVars.Parameters: self.NmpcVars.Hp, self.NmpcVars.Hc, self.NmpcVars.Q, self.NmpcVars.R, self.NmpcVars.Ts, self.PathError.c, self.PathError.sp

		self.J_min =self.NmpcVars.Result[0]
		self.u_opt[0]=self.NmpcVars.Result[1]
		self.u_opt[1]= self.NmpcVars.Result[2]
		self.u_opt[2]=self.NmpcVars.Result[2]

	def TPathFollowing_Act(self):
		vx_path,vy_path,ang,vx,vy,x = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
		f1 = 0
		arg,w = 0.0, 0.0

		self.PathError.u1[0]=self.u_opt[0]
		self.PathError.u2[0]=self.u_opt[1]
		self.PathError.u3[0]=self.u_opt[2]

		print("patherror_u1 = ") + str(self.PathError.u1) + ("patherror_u2 = ") + str(self.PathError.u2)

		arg=self.PathError.u2[0]/self.PathError.vnav

		if abs(arg)<=1:
			self.PathError.thetae[1]=math.asin(arg)

		else:
			if arg>0:
				self.PathError.thetae[1]=math.pi/2

			else:
				self.PathError.thetae[1]=-math.pi/2

		self.PathError.sp=self.PathError.vnav*math.cos(self.PathError.thetae[0])-self.PathError.u1[0]
		self.PathError.xe[0]=self.PathError.xe[0]+self.Ts*((self.PathError.ye[0]*self.PathError.c*self.PathError.sp)-(self.PathError.sp)+(self.PathError.vnav*math.cos(self.PathError.thetae[1])))
		self.PathError.ye[0]=self.PathError.ye[0]+self.Ts*(-self.PathError.xe[0]*self.PathError.c*self.PathError.sp+self.PathError.vnav*math.sin(self.PathError.thetae[1]))

		self.PathError.s=self.PathError.s+self.NmpcVars.Ts*self.PathError.sp

		self.RobotReference.v=self.PathError.vnav
		w=((self.PathError.thetae[1]-self.PathError.thetae[0])/(self.Ts))+self.PathError.c*self.PathError.sp
		#//w=self.PathError.c*self.PathError.sp

		self.RobotReference.w=w

		print("w reference = ") + str(w)

		velocities = Twist()
		velocities.linear.x = self.RobotReference.v
		velocities.angular.z = self.RobotReference.w
		self.cmdVelPub.publish(velocities)
		print("velocities = ") +str(velocities)

	def OdomCb(self,robot_state):
		# transform from quaternion for yaw degrees
		quaternion = (robot_state.pose.pose.orientation.x, robot_state.pose.pose.orientation.y, robot_state.pose.pose.orientation.z, robot_state.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		yaw = euler[2]
		self.robot_pose = [robot_state.pose.pose.position.x,robot_state.pose.pose.position.y, yaw]

		#por que isso aqui?
		self.v = robot_state.twist.twist.linear.x
		self.w = robot_state.twist.twist.angular.z

		self.RobotState.x = self.robot_pose[0]
		self.RobotState.y = self.robot_pose[1]
		self.RobotState.teta = self.robot_pose[2]
		print("robot state = ") +str(self.RobotState)

		if (self.z == 0):
			self.Ti = rospy.get_time()
			self.z = 1

		if (self.Tf-self.Ti >= 0.2):
			self.z = 0
			self.TPathFollowing_Predictor()
			self.TPathFollowing_Act()

		self.Tf = rospy.get_time()


		self.TPathFollowing_GetReference()
		self.TPathFollowing_CallOptimizer()


	def optimizerCb(self, data):
		self.J_min = data.optimizerResult[0]
		self.last_opt = data.optimizerResult[1]

		self.J_min =data.optimizerResult[0]
		self.u_opt[0]=data.optimizerResult[1]
		self.u_opt[1]= data.optimizerResult[2]

		#quem tratar aqui???
		#self.wRef = (self.last_opt*math.cos(self.thetae[0])+self.circunferenceCurvature*self.vRef)/((math.cos(self.thetae[0])-self.circunferenceCurvature*self.de[0]))
		velocities = Twist()
		velocities.linear.x = self.RobotReference.v
		velocities.angular.z = self.RobotReference.w

		#self.cmdVelPub.publish(velocities)

	def deviationCb(self, data):
		if data.status == 0:
			self.useVisualControl = False

		else:
			self.useVisualControl = True


def main():
	rospy.init_node('path_following')
	path_obj = PathFollowing()

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep


if __name__ == '__main__':
	main()
