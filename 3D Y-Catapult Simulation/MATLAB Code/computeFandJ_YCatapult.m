
%% Compute the elastic forces & Energy

for c=1:TotalN-1
        
    if(c == N_inc) || (c==2*N_inc) || c==(2*N_inc+N_v)
        continue
    end
    
    if (c < 2*N_inc)
        dl = dl_inc;
        EA_temp = EA;
    elseif (c>2*N_inc && c<(2*N_inc+N_v))
        dl = dl_v;
        EA_temp = EA;
    else
        dl = dl_string;
        EA_temp = EA_string;
    end

    node0 = [q_new(3*c-2), q_new(3*c-1), q_new(3*c)];
    node1 = [q_new(3*c+1), q_new(3*c+2), q_new(3*c+3)];
    ind = [3*c-2, 3*c-1, 3*c, 3*c+1, 3*c+2, 3*c+3];
    
    [dF, dJ] = gradEs_hessEs(node0,node1,dl,EA_temp); %tempFES contains gradient of node k and k+1
    tempFES(ind) = dF;
    tempjES(ind, ind) = dJ; 
    fES = fES + tempFES;
    jES = jES + tempjES; 
    tempFES = zeros(3*TotalN,1);
    tempjES = zeros(3*TotalN,3*TotalN); %setting this to zero after end of loop so it can store new values.
end

%% Computing bending Force and Hessin

for c=2:TotalN-1
    
    if(c == N_inc) || (c==2*N_inc) || (c == N_inc+1) || (c==2*N_inc+1) || c==(2*N_inc+N_v) || c==(2*N_inc+N_v+1)
        continue
    end

    dl = voronoiLength(c);

    node0 = [q_new(3*c-5), q_new(3*c-4), q_new(3*c-3)];
    node1 = [q_new(3*c-2), q_new(3*c-1), q_new(3*c)];
    node2 = [q_new(3*c+1), q_new(3*c+2), q_new(3*c+3)];
    ind = [3*c-5, 3*c-4, 3*c-3, 3*c-2, 3*c-1, 3*c, 3*c+1, 3*c+2, 3*c+3];
    curvature0= 0;
    if(c>(2*N_inc+N_v))
        EI_temp = EI_string;
    else
        EI_temp = EI;
    end

    [dF, dJ] = gradEb_hessEb3D(node0, node1, node2, curvature0, dl, EI_temp); %tempFES contains gradient of node k and k+1
    tempFEB(ind) = dF;
    tempjEB(ind, ind) = dJ; 
    fEB = fEB + tempFEB;
    jEB = jEB + tempjEB; 
    tempFEB = zeros(3*TotalN,1);
    tempjEB = zeros(3*TotalN,3*TotalN); %setting this to zero after end of loop so it can store new values.
end

%% Computing Natural Curvature
node0_T0 = [q0(4:6)]; %original position of second node of left rod at T=0
node1_T0 = [q0(1:3)]; %original position of common node1 at T=0
node2_T0 = [q0(6*N_inc+4: 6*N_inc+6)]; %original position of second node of straight rod at t=0
node3_T0 = [q0(3*N_inc+4: 3*N_inc+6)]; %original position of second node of right rod at t=0

posNode4 = 3*TotalN - 3*N_string+1; %starting of common node2
posNode5 = numel(q0)-2; %starting indice of common node3

node4_T0 = [q0(posNode4 : posNode4+2)]; %indice of common node2
node5_T0 = [q0(posNode5:  posNode5+2)]; %indice of common node3

posNode6 = 3*N_inc-5; %indice of second last node of left rod
posNode7 = 6*N_inc-5; %indice of second last node of right rod

node6_T0 = [q0(posNode6 : posNode6+2)]; % Indices of second last node of left rod
node7_T0 = [q0(posNode7 : posNode7+2)]; %indice of second last node of right rod

node8_T0 = [q0(posNode4+3 : posNode4+5)]; %second node of string
node9_T0 = [q0(posNode5-3 : posNode5-1)]; %second last node of string


%% Calculating Bending force and hessin at the joint of Y Shape
    node0 = [q_new(4), q_new(5), q_new(6)]; %second node of left rod
    node1 = [q_new(1), q_new(2), q_new(3)]; %Common node     
    node2 = [q_new(6*N_inc+4), q_new(6*N_inc+5), q_new(6*N_inc+6)]; %Second node of straight rod
    ind = [4, 5, 6, 1, 2, 3, 6*N_inc+4, 6*N_inc+5, 6*N_inc+6];
    dl = voronoiLength(1);
    curvature0 = computekappa(node0_T0, node1_T0,node2_T0);
    [dF, dJ] = gradEb_hessEb3D(node0, node1, node2, curvature0, dl, EI);
    fEB(ind) = fEB(ind) + dF;
    jEB(ind,ind) = jEB(ind,ind) + dJ; 

    node3 = [q_new(3*N_inc+4), q_new(3*N_inc+5), q_new(3*N_inc+6)]; %second node of right rod   
    ind = [3*N_inc+4, 3*N_inc+5, 3*N_inc+6,1,2,3,6*N_inc+4, 6*N_inc+5, 6*N_inc+6];

    curvature0 = computekappa(node3_T0, node1_T0, node2_T0);
    [dF, dJ] = gradEb_hessEb3D(node3, node1, node2, curvature0, dl, EI);
    fEB(ind) = fEB(ind) + dF;
    jEB(ind,ind) = jEB(ind,ind) + dJ; 

    curvature0 = computekappa(node0_T0, node1_T0, node3_T0); %compting bending energy of left, center and right node
    ind = [4,5,6, 1,2,3, 3*N_inc+4, 3*N_inc+5, 3*N_inc+6];
    [dF, dJ] = gradEb_hessEb3D(node0, node1, node3, curvature0, dl, EI);
    fEB(ind) = fEB(ind) + dF;
    jEB(ind,ind) = jEB(ind,ind) + dJ; 

    %% Computing Bending force and hession at joint 2 and 3

    EI_temp = (EI_string + EI)/2 ;
    node4 = [q_new(posNode4:posNode4+2)]'; %common node2
    node6 = [q_new(posNode6:posNode6+2)]'; %second last node of left rod
    node8 = [q_new(posNode4+3 : posNode4+5)]'; %second node of string
    ind = [posNode6:posNode6+2, posNode4:posNode4+2, posNode4+3 : posNode4+5];
    dl = voronoiLength(N_inc);
    curvature0 = computekappa(node6_T0, node4_T0,node8_T0);
    [dF, dJ] = gradEb_hessEb3D(node6, node4, node8, curvature0, dl, EI_temp);
    fEB(ind) = fEB(ind) + dF;
    jEB(ind,ind) = jEB(ind,ind) + dJ; 

    node5 = [q_new(posNode5 : posNode5+2)]'; %common node3
    node7 = [q_new(posNode7 : posNode7+2)]'; %second last node of right rod
    node9 = [q_new(posNode5-3 : posNode5-1)]'; %second last node of string
    ind = [posNode5-3 : posNode5-1, posNode5:  posNode5+2, posNode7 : posNode7+2];
    dl = voronoiLength(2*N_inc);
    curvature0 = computekappa(node9_T0, node5_T0, node7_T0);
    [dF, dJ] = gradEb_hessEb3D(node9, node5, node7, curvature0, dl, EI_temp);
    fEB(ind) = fEB(ind) + dF;
    jEB(ind,ind) = jEB(ind,ind) + dJ; 


 %% Equating force and hession to all common indice for joint 1 and 2
posNode10 = [3*N_inc-2,3*N_inc-1,3*N_inc]'; %last node of left rod
posNode11 = [6*N_inc-2, 6*N_inc-1, 6*N_inc]'; %last node of right rod

TfEBL = fEB(posNode4:posNode4+2) + fEB(posNode10);
TfEBR = fEB(posNode5:posNode5+2) + fEB(posNode11);
TfESL = fES(posNode4:posNode4+2) + fES(posNode10);
TfESR = fES(posNode5:posNode5+2) + fES(posNode11);

TjEBL = jEB(posNode4:posNode4+2,:) + jEB(posNode10,:);
TjEBR = jEB(posNode5:posNode5+2,:) + jEB(posNode11,:);
TjESL = jES(posNode4:posNode4+2,:) + jES(posNode10,:);
TjESR = jES(posNode5:posNode5+2,:) + jES(posNode11,:);

fEB(posNode10) = TfEBL; fEB(posNode4:posNode4+2) = TfEBL;
fEB(posNode11) = TfEBR; fEB(posNode5:posNode5+2) = TfEBR;

fES(posNode10) = TfESL; fES(posNode4:posNode4+2) = TfESL;
fES(posNode11) = TfESR; fES(posNode5:posNode5+2) = TfESR;

jEB(posNode10,:) = TjEBL; jEB(posNode4:posNode4+2,:) = TjEBL;
jEB(posNode11,:) = TjEBR; jEB(posNode5:posNode5+2,:) = TjEBR;

jES(posNode10,:) = TjESL; jES(posNode4:posNode4+2,:) = TjESL;
jES(posNode11,:) = TjESR; jES(posNode5:posNode5+2,:) = TjESR;


%% Compute total force at the joint. Equating to all common indices.
node0 = [1,2,3]; node1 = [3*N_inc+1,3*N_inc+2,3*N_inc+3]; node2= [6*N_inc+1, 6*N_inc+2,6*N_inc+3];
TfEB = fEB(node0) + fEB(node1) + fEB(node2);
TfES = fES(node0) + fES(node1) + fES(node2);
TjEB = jEB(node0, :) + jEB(node1, :) + jEB(node2,:);
TjES = jES(node0,:) + jES(node1,:) + jES(node2,:);

fEB(node0) = TfEB; fES(node0) = TfES;
fEB(node1) = TfEB; fES(node1) = TfES;
fEB(node2) = TfEB; fES(node2) = TfES;

jEB(node0, :) = TjEB;
jEB(node1, :) = TjEB;
jEB(node2, :) = TjEB;

jES(node0, :) = TjES;
jES(node1, :) = TjES;
jES(node2, :) = TjES;


f = M / dt * ((q_new- q_old)/dt - uq_old) + fEB + fES - 6*pi*mu*r_out*(q_new- q_old)/dt;

%if(k<2)
 %   f(3*N_inc-1) = f(3*N_inc-1) + 2000;
  %  f(6*N_inc-1) = f(6*N_inc-1) + 200;
%end

J = M / dt^2 + jEB + jES - 6*pi*mu*r_out/dt;


fES = zeros(3*TotalN,1); %empty the FEB and FES so that they can store new loop values
fEB = zeros(3*TotalN,1);
jEB = zeros(3*TotalN,3*TotalN);
jES = zeros(3*TotalN,3*TotalN); %empty the FEB and FES so that they can store new loop values
