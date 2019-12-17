L_1=220;L_2=270;
a=0;b=0;c=0;
theta1=(deg2rad(a) ) ; theta2=(deg2rad(b) ) ; theta3=(deg2rad(c) ) ;
T1=transl (0 ,0 ,237.6)*trotz ( theta1 ) ;
T2=transl (0 ,0 ,0)*trotx (deg2rad(90))*trotz (deg2rad(0)+theta2 ) ;
T3=transl (216 ,0 ,0)*trotz (deg2rad(0)+theta3 ) ;
T4=transl (206 ,0 ,0)*trotx (deg2rad(90) ) ;
Tb_w=T1*T2*T3*T4
Tb_w2=T1*T2*T3;

Vec_pos=[Tb_w(1 ,4) ;Tb_w(2 ,4) ;Tb_w(3 ,4) ]; 
Vec1_4=Vec_pos-[T1(1 ,4) ;
T1(2 ,4) ;T1(3 ,4) ];

thet1=atan2(Vec1_4(2),Vec1_4(1)) ;
thetas1=(rad2deg( thet1 ) )

L_3=sqrt ((Vec1_4(3 ,1) )^2+(Vec1_4(1 ,1) )^2+(Vec1_4(2 ,1) )^2) ;

f_11=acos ((L_1^2+L_3^2-L_2^2)/(2*L_3*L_1) ) ;
f_22=atan2(Vec1_4(3 ,1) ,( sqrt ((Vec1_4(1 ,1) )^2+(Vec1_4(2 ,1) )^2)) ) ;
fi_1=(rad2deg (( f_11 ) ) ) ;
fi_2=(rad2deg( f_22 ) ) ;
thetas2_1=-(fi_2-fi_1 )
thetas2_2=-(fi_2+fi_1 )

f_33=acos ((L_1^2+L_2^2-L_3^2)/(2*L_1*L_2) ) ;
theta3_1=-180+(rad2deg( f_33 ) )
theta3_2=180-(rad2deg( f_33 ) )
