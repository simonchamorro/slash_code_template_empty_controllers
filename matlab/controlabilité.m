clear all
clc
close all

mat_a=[1,2;3,4]
mat_b=[2.4;6.8]

mat_controlabilite=[mat_b,mat_a*mat_b]

if(det(mat_controlabilite)==0)
    disp("Le syst�me n'est pas contr�lable")

else
    disp("Le syst�me est contr�lable")
    
end

    