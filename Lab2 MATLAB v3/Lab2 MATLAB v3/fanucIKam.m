function [is_solution,joint_angles] = fanucIKam(T,prev_joint_angles,fanuc)
l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_6 = fanuc.parameters.l_6;
a_3 = fanuc.parameters.a_3;
%rename parameters

positionvector=T(1:3,4);
positionvector=positionvector+T(1:3,1:3)*[0;0;-l_6];

px=positionvector(1);
py=positionvector(2);
pz=positionvector(3);
workspace=fanuc.workspace;

%solving theta1 theta2 theta3 using geometric method 
if px==0 && py==0
    theta1sol=prev_joint_angles(1);
    
else
    theta1sol=atan2(py,px);
end
x=(sqrt(px^2+py^2)-300);
y=pz-1000;
beta=atan2(y,x);
l_4offset=sqrt(l_4^2+180^2);

fi=acos((x^2+y^2+l_3^2-l_4offset^2)/2/l_3/sqrt(x^2+y^2));
if prev_joint_angles(2)>beta
    theta2sol=beta+fi;
    theta3sol=3.141592653589/2-acos((x^2+y^2-l_3^2-...
        l_4offset^2)/2/l_3/l_4offset)-atan2(180,l_4);
    if theta2sol>170/180*3.141592653589
        theta2sol=beta-fi;
        theta3sol=-theta3sol;
    end
    
    
else
    theta2sol=beta-fi;
    theta3sol=-3.141592653589/2+acos((x^2+y^2-l_3^2-...
        l_4offset^2)/2/l_3/l_4offset)+atan2(180,l_4);
    if theta2sol<10/180*3.141592653589
        theta2sol=beta+fi;
        theta3sol=-theta3sol;
    end
end
%generate the rotation matrix from frame 4 to frame 6
T10n=dhtf(0, 0, 0, theta1sol);
R10=T10n(1:3,1:3);
T21n=dhtf(3.141592653589/2, l_2, 0, theta2sol);
R21=T21n(1:3,1:3);
T32n=dhtf(0, l_3, 0, theta3sol);
R32=T32n(1:3,1:3);
T43theta4null=dhtf(3.141592653589/2, a_3, l_4, 0);
R43theta4null=T43theta4null(1:3,1:3);

R40theta4null=R10*R21*R32*R43theta4null;
R64=transpose(R40theta4null)*T(1:3,1:3);

%calculating and selecting theta4 theta5 theta6
if isreal([theta1sol, theta2sol, theta3sol])==1
    theta5sol=atan2(sqrt(R64(3,1)^2+R64(3,2)^2),R64(3,3));
    if sin(theta5sol)~=0
        theta4sol=atan2(R64(2,3)/sin(theta5sol),R64(1,3)/sin(theta5sol));
        theta6sol=atan2(R64(3,2)/sin(theta5sol),-R64(3,1)/sin(theta5sol));
    else if theta5sol==0
            theta4sol=0;
            theta6sol=atan2(-R64(1,2),R64(1,1));
        else
            theta4sol=0;
            theta6sol=atan2(R64(1,2),-R64(1,1));
        end
    end
    
    
    if theta4sol>-2/3*3.141592653589 && theta4sol<2/3*3.141592653589
        
    else
        if prev_joint_angles(4)>0
            if theta4sol<0
                theta4sol=2*3.141592653589+theta4sol;
            end
        else
            if theta4sol>0
                theta4sol=-2*3.141592653589+theta4sol;
            end
        end
    end
    
    if theta6sol>-3.141592653589/3 && theta6sol<3.141592653589/3
    else
        
        if prev_joint_angles(6)>0
            if theta6sol<0
                theta6sol=2*3.141592653589+theta6sol;
            end
        else
            if theta6sol>0
                theta6sol=-2*3.141592653589+theta6sol;
            end
        end
    end
    %decide if the joint angles are withing limits
    joint_angles=[theta1sol theta2sol theta3sol theta4sol theta5sol theta6sol];
    if isreal(joint_angles)==1
        if theta1sol>fanuc.joint_limits{1}(1) && theta1sol<fanuc.joint_limits{1}(2)...
                && theta2sol>fanuc.joint_limits{2}(1) && theta2sol<fanuc.joint_limits{2}(2)...
                && theta3sol>fanuc.joint_limits{3}(1) && theta3sol<fanuc.joint_limits{3}(2)...
                && theta4sol>fanuc.joint_limits{4}(1) && theta4sol<fanuc.joint_limits{4}(2)...
                && theta5sol>fanuc.joint_limits{5}(1) && theta5sol<fanuc.joint_limits{5}(2)...
                && theta5sol>fanuc.joint_limits{6}(1) && theta5sol<fanuc.joint_limits{6}(2)
            %make is_solution true when there is a solution 
            is_solution=true;
        else
            is_solution=false;
            joint_angles=prev_joint_angles;
            
            
        end
    else
        %if cannot find a solution, make the joint angle remain to be
        %previous joint angles and is_solution to be false
        is_solution=false;
        joint_angles=prev_joint_angles;
    end
else
    is_solution=false;
    joint_angles=prev_joint_angles;
    
end

end