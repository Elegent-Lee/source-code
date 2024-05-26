function out = odeRhs(~,odeVec,settings,simObj,counter) 

%Copy settings into local variables
NSpecies = settings.NSpecies;
NAgents = settings.NAgents;
TotAgent = size(odeVec,1) / 4;
agents = reshape(odeVec,TotAgent,4);
r = settings.r;
d = settings.d;
mass = settings.mass;
h = settings.h;% 
Xi = settings.Xi;%
LFM = settings.LFM;

[mesh1X, mesh2X] = meshgrid(agents(:,1),agents(:,1));
[mesh1Y, mesh2Y] = meshgrid(agents(:,2),agents(:,2));
distanceMatrixX = mesh2X-mesh1X; % distance matrix in x direction
distanceMatrixY = mesh2Y-mesh1Y; % distance matirx in y direction
distanceCellX = mat2cell(distanceMatrixX,NAgents,NAgents);
distanceCellY = mat2cell(distanceMatrixY,NAgents,NAgents);

[mesh1VX, mesh2VX] = meshgrid(agents(:,3),agents(:,3));
[mesh1VY, mesh2VY] = meshgrid(agents(:,4),agents(:,4));
velocityMatrixX = mesh2VX-mesh1VX; % distance matrix in x direction
velocityMatrixY = mesh2VY-mesh1VY; % distance matirx in y direction
velocityCellX = mat2cell(velocityMatrixX,NAgents,NAgents);
velocityCellY = mat2cell(velocityMatrixY,NAgents,NAgents);

%-------------------------------Calc u_g and u_d-----------------------------%
u_alpha_temp = cell(size(distanceCellX));
u_alpha_temp_2 = cell(length(distanceCellX),1);
   
    
    for k = 1:length(distanceCellX)
    for l = 1:length(distanceCellX)
        for i = 1:length(distanceCellX{l,k}(1,:))
            fi_g = zeros(1,2); %Gradient based term
            fi_d = zeros(1,2); %velocity consensus term
            for j = 1:length(distanceCellX{l,k}(:,1))
                if k == l    
                    if ((j~=i) && (norm([distanceCellX{l,k}(j,i),distanceCellY{l,k}(j,i)]) < r(k)))
                        fi_g(1,:) = fi_g + (Xi(i)*Morseatt(norm([distanceCellX{l,k}(j,i),distanceCellY{l,k}(j,i)]),d(k),r(k))+Morsere(norm([distanceCellX{l,k}(j,i),distanceCellY{l,k}(j,i)]),d(k),r(k)))*gradient_sigma_norm([distanceCellX{l,k}(j,i),distanceCellY{l,k}(j,i)]);
                        fi_d(1,:) = fi_d + adj_matrix([distanceCellX{l,k}(j,i),distanceCellY{l,k}(j,i)],r(k))*[velocityCellX{l,k}(j,i),velocityCellY{l,k}(j,i)];
                    end
                end
            end
            fi_g = 0.5*fi_g;
            fi_d = 0.5*fi_d;
            u_alpha_temp{l,k}(i,:) = fi_g + fi_d;
        end
    end
    end



for k = 1:length(u_alpha_temp(1,:))
    temp = zeros(size(u_alpha_temp{1,k}));
    for j = 1:length(u_alpha_temp(:,1))
        temp =  temp + u_alpha_temp{j,k};
    end
    u_alpha_temp_2{k,1} = temp;
end

u_alpha = cell2mat(u_alpha_temp_2);
  
    
if strcmp(settings.SimMode, 'algorithm1')
    u = u_alpha;

elseif strcmp(settings.SimMode, 'algorithm2') || strcmp(settings.SimMode, 'algorithm3')
    
    %copy settings into local variables
    gamma_agent = simObj.gamma;
    qd = gamma_agent(1:2);
    pd = gamma_agent(3:4);
    Gamma_c1 = settings.Gamma_c1;
    Gamma_c2 = settings.Gamma_c2;
    Gamma_c3 = settings.Gamma_c3;
    Gamma_c4 = settings.Gamma_c4;
    force_mag = 3;
    
%----------------------------Calc u_gamma u_l ------------------------------%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   u-l  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NN=zeros(NAgents,2); 
for i=1:NAgents
     count=0;
for j=1:NAgents %For every
    if j~=i %non-self particle
        if (agents(i,1)-agents(j,1))^2+(agents(i,2)-agents(j,2))^2<r(1)^2 %If particle j within distance R from particle i
                count=count+1;
                NN(count,:)=[agents(j,1),agents(j,2)];
                NN1(count,:)=[agents(j,3),agents(j,4)];
        end
    end
end
NN=NN(1:count,:);
sNN=size(NN,1);
if sNN>0    
 
    CMx=mean(NN(:,1));
    CMy=mean(NN(:,2));
    CMvx=mean(NN1(:,1));
    CMvy=mean(NN1(:,2));
    
    delta_pos(i,:) = [( CMx-agents(i,1)),(CMy-agents(i,2))]; 
    delta_pos(i,:) = delta_pos(i,:)/norm(delta_pos(i,:));
    delta_vel(i,:) = [( CMvx-agents(i,3)),(CMvy-agents(i,4))];
else
    delta_pos(i,:) = [0,0];
    delta_vel(i,:) = [0,0];
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Agent pos and vel -  gamma agent pos and vel
    delta_pos2 = [(agents(:,1) - repmat(qd(1),TotAgent,1)),(agents(:,2) - repmat(qd(2),TotAgent,1))];
    delta_vel2 = [(agents(:,3) - repmat(pd(1),TotAgent,1)),(agents(:,4) - repmat(pd(2),TotAgent,1))];
    
    %Repeat gamma constants (NAgents) number of times
    Gamma_c1_rep = repmat(repeat(NAgents,Gamma_c1)',1,2);
    Gamma_c2_rep = repmat(repeat(NAgents,Gamma_c2)',1,2);
    Gamma_c3_rep = repmat(repeat(NAgents,Gamma_c3)',1,2);
    Gamma_c4_rep = repmat(repeat(NAgents,Gamma_c4)',1,2);
    
    u_l = Gamma_c3_rep.*delta_pos + Gamma_c4_rep.*delta_vel;% local feedback mechanism
    u_gamma2 = -Gamma_c1_rep.*delta_pos2 - Gamma_c2_rep.*delta_vel2;% u_gamma
    u_gamma=LFM*u_l+h.*u_gamma2;
%-------------------------Calc u_rand--------------------------------%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

u = u_alpha + u_gamma ;

else
    error('Set mode to either algorithm1, algorithm2 or algorithm3');
end
    
if strcmp(settings.SimMode, 'algorithm3')
  %Calculate and add u_beta if algorithm 3
  obstacles = simObj.obstacles;
  NObstacles = length(obstacles(:,1));
  r_prime = settings.r_prime;
  d_prime = settings.d_prime;
  Beta_c1 = settings.Beta_c1;
  Beta_c2 = settings.Beta_c2;
  Agents = simObj.agents; %Different from agents used above. Above is array.
  u_beta = cell(size(Agents));
  
  
  for i = 1:length(Agents)
      for j = 1:length(Agents{1,i}(:,1))
          fi_g = zeros(1,2); %Gradient based term
          fi_d = zeros(1,2); %velocity consensus term
          for k = 1:NObstacles
              if norm(Agents{1,i}(j,1:2) - obstacles(k,1:2)) <= (r_prime(i)+obstacles(k,3)) %vicinity check. If obstacle within agent's obstacle sensing range
                  mu_beta = obstacles(k,3)/norm(Agents{1,i}(j,1:2) - obstacles(k,1:2)); %page 40 of paper
                  a_beta = (Agents{1,i}(j,1:2) - obstacles(k,1:2))/norm(Agents{1,i}(j,1:2) - obstacles(k,1:2));
                  P_beta = eye(2) - a_beta'*a_beta;
                  q_beta = mu_beta*Agents{1,i}(j,1:2) + (1-mu_beta)*obstacles(k,1:2);
                  p_beta = (mu_beta*P_beta*Agents{1,i}(j,3:4)')';
                  fi_g = fi_g + Beta_c1(i)*phi_alpha(sigma_norm(q_beta - Agents{1,i}(j,1:2)),d_prime(i),r_prime(i))*gradient_sigma_norm(q_beta - Agents{1,i}(j,1:2));
                  fi_d = fi_d + Beta_c2(i)*adj_matrix(q_beta - Agents{1,i}(j,1:2),d_prime(i))*(p_beta-Agents{1,i}(j,3:4));
              end
          end
          u_beta{1,i}(j,:) = fi_g + fi_d;
      end
  end
  
  u_beta = cell2mat(u_beta');
  
  u = u + u_beta;
end

agentForceVecX = u(:,1);
agentForceVecY = u(:,2);

%Repeat Mass (NAgents) number of times
mass_rep = repeat(NAgents,mass)';

out = [odeVec(2*TotAgent+1:4*TotAgent);agentForceVecX./mass_rep;agentForceVecY./mass_rep];

end

function out = repeat(r,x)
    t = r > 0;
    a = cumsum(r(t));
    b = zeros(1,a(end));
    b(a - r(t) + 1) = 1;
    x1 = x(t);
    out = x1(cumsum(b));
end
