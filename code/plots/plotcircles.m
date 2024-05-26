function h = plotcircles(agents)

for i=1:length(agents)-1
    for j=1:length(agents{1,i})
        d = agents{1,2}(1)*2;
        h(j,:) = rectangle('Position',[agents{1,i}(j,1)-agents{1,2}(1), agents{1,i}(j,2)-agents{1,2}(1), d, d],'Curvature',[1,1], 'FaceColor', 'none', 'edgeColor','k','LineWidth',0.01,'Linestyle','--');
    end
end
% h = rectangle('Position',[x-r, y-r, d, d],'Curvature',[1,1], 'FaceColor', 'none', 'edgeColor','r','LineWidth',1);
end
