function mainfunc2()

% load or generate settings
if exist('presets/Settings.mat', 'file') == 2         %  xx
    if sum(strcmp(who('-file', 'presets/Settings.mat'), 'settings')) == 1
        load('presets/Settings.mat', 'settings');
    else
        settings = createDefaultSettings();
    end                              %    xx
else
    settings = createDefaultSettings();
end

% generates simulationObj (agents and in future obstacles) in depending on settings
simulationObj = createSimulationObj(settings);


%set time for simulation and plot
simulationObj = resetSimulationObj(simulationObj);
  
  % Open up the figure
figure2 = figure(2);

% create avi-file
% if (settings.captureBool)   %  xx
%     open(videoObj);
% end          %  xx

% plot everything
plotObj = plotInit(simulationObj, settings, figure2);

% create timerfunction
period = settings.period; %Time interval between timerFcn calls
num_tasks = settings.iteration; %Number of times to repeat timerFcn
timerFcn = @(~,~) timerFunction();

timerObj = timer('TimerFcn', timerFcn, ...
    'ExecutionMode', 'fixedRate', ...
    'Period', period, 'TasksToExecute', num_tasks, ...
    'BusyMode' , 'queue');


%counter
% global counter
counter = 0;

%Start timer
start(timerObj);

%Pause for tasks to execute
pause(num_tasks*period);

%Stop timer
 stop(timerObj);



%Delete timer
 delete(timerObj);
 

%Nested timer function to use the objects without passing any variables
%TIMERFUNCTION calls the updateAgent function and plots the new matrix

 z=[];x=[];z1=[];x1=[];z2=[];x2=[];Num1=[];Numf={};Numfatt=[];Numd=[];
function timerFunction()
    tic;
    

     
 %Get agent steps and update plot
    simulationObj = agentsStep(simulationObj, settings,counter);
    plotUpdate(plotObj,simulationObj,settings);
    drawnow
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


 ii=counter+1;
    for j=1:settings.NAgents(1,1)
        z1(j,ii)=simulationObj.agents{1,1}(j,1);
        x1(j,ii)=simulationObj.agents{1,1}(j,2);
        hold on
    end



    % Display agents position and vel and iterations for debugging
    %disp(simulationObj.agents);
    counter = counter +1;
    t_iter_end = toc;   
    fprintf('Step = %d \n', counter);
%%%%%%%%%%%%%%%%%%%%O_a, O_d, O_N%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d=settings.d;
r=settings.r;
Xi=settings.Xi;

gamma_agent = simulationObj.gamma;
for i1=1:settings.NAgents
     count=0;
for j1=1:settings.NAgents %For every
    if j1~=i1 %non-self 
        if (simulationObj.agents{1,1}(i1,1)-simulationObj.agents{1,1}(j1,1))^2+(simulationObj.agents{1,1}(i1,2)-simulationObj.agents{1,1}(j1,2))^2<settings.r(1)^2 %If agent j within distance r agent particle i
                count=count+1;
        end
                F(i1,j1)=Xi(i1)*Morseatt(norm([simulationObj.agents{1,1}(i1,1)-simulationObj.agents{1,1}(j1,1),simulationObj.agents{1,1}(i1,2)-simulationObj.agents{1,1}(j1,2)]),5,7);
    end
end
Num(i1,:) = count;
Num1(:,counter)=mean(Num,1);
Numf{1,counter}=F;
end
for i2=1:counter
    kkk(i2,:)=sum(sum(Numf{1,i2}));
    Numfatt(:,i2)=sum(sum(kkk(i2,:)))/2;
end
Numd(:,counter)=norm([mean(simulationObj.agents{1,1}(:,1),1)-gamma_agent(1),mean(simulationObj.agents{1,1}(:,2),1)-gamma_agent(2)]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
if  (counter == settings.iteration)    
  z=[z1]; x=[x1]; Num1=[Num1];
  %x,y-position images
  figure(3);  
    for q=1:sum(settings.NAgents ,2) 
        plot(0:length(z)-1,z(q,:))
         set(gca,'looseInset',[0 0 0 0]);
        hold on;
    end
    ylabel('x-position of the agents');xlabel('Step');
      
     figure(4);       
    for w=1:sum(settings.NAgents ,2)
         plot(0:length(z)-1,x(w,:))
          set(gca,'looseInset',[0 0 0 0]);
         hold on;
    end
    ylabel('y-position of the agents');xlabel('Step');
     

   
   figure(5);
    plot(0:settings.iteration-1,Num1,'color',[1.00,0.75,0.48],'linewidth',2)
     set(gca,'looseInset',[0 0 0 0]);
    hold on;
   plot(0:settings.iteration-1,Numfatt,'color',[0.55,0.81,0.79],'linewidth',2)
     set(gca,'looseInset',[0 0 0 0]);
     hold on;
   plot(0:settings.iteration-1,Numd,'color',[0.51,0.69,0.82],'linewidth',2)
     set(gca,'looseInset',[0 0 0 0]);
     hold on;
    ylabel('Criteria for evaluation');xlabel('Step'); 
    legend({'O_N','O_a','O_d'},'Location','northwest')

   
    figure(8);   
    for n=1:sum(settings.NAgents ,2)
        plot(z(n,1),x(n,1),'k.','MarkerSize',10) 
     set(gca,'looseInset',[0 0 0 0]);
        hold on;
         plot(z(n,end),x(n,end),'b.','MarkerSize',15)
        set(gca,'looseInset',[0 0 0 0]); 
        hold on;
        plot(z(n,1:end),x(n,1:end))
        set(gca,'looseInset',[0 0 0 0]);
        hold on;
    end          
%  
   z_1=sum(z)./sum(settings.NAgents ,2);% avg-dis
   x_1=sum(x)./sum(settings.NAgents ,2);
   for o=1:20
        plot(z_1(1,o),x_1(1,o),'m.')
   end  
   hold on;
    for o=21:counter
        plot(z_1(1,o),x_1(1,o),'m.')
    end
  hold on;  
  
 end
  

end  
end 
    
    
