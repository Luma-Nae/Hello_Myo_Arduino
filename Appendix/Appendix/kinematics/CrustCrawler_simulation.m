%CrustCrawler Kinematics simulation
%D&H parameters of the CrustCrawler
clear all ; close all ; clc;
L0 = Link( 'alpha' , 0, 'a' , 0, 'd' , 0, 'modified' ) ; %Extra link added to simulate the table
LB = Link( 'alpha' , 0, 'a' , 0, 'd' , 171 , 'modified') ; %From bottom of robot to the first joint
L1 = Link( 'alpha' , (deg2rad(90)) , 'a' , 0, 'd' , 0, 'modified') ; %From first joint to second join
L2 = Link( 'alpha' , 0, 'a' , 220, 'd' , 0, 'modified') ; %From second joint to third joint
L3 = Link( 'alpha' , (deg2rad(-90)) , 'a' , 270, 'd' , 0, 'modified') ; %From third joint to end effector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Transformation matrices of the CrustCrawler
Tb_1=transl (0 ,0 ,171) 
T1_2=transl (0 ,0 ,0)*trotx (deg2rad(90))
T2_3=transl (220 ,0 ,0)
T3_4=transl (270 ,0 ,0)*trotx (deg2rad(-90) )
T=Tb_1*T1_2*T2_3*T3_4 %Final transformation matrix

CrustCrawler=SerialLink ([L0 LB L1 L2 L3] , 'name' , 'CrustCrawler') ;
q=[0 ,0 ,0 ,0 ,0];
TT=CrustCrawler.fkine(q) %Beginning position of the joints

CrustCrawler.teach(q)