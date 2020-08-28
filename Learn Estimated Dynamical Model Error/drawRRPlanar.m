function drawRRPlanar(t,x,model)

    length = model.l1 + model.l2;
    axis equal; axis(length*[-1,1,-1,1]); axis off;

    [p1,p2] = RRPlanarFKine(model,x);
    pos = [[0;0],p1,p2];

    %%%% Draw base
    base_size = model.l1 * 0.4;
    % 
    rectangle('Position', [-base_size/2 -base_size/2 base_size base_size],...
        'Curvature', 0.4,...
        'LineWidth', 0.2,...
        'FaceColor', [1 1 1]*0.7,...
        'EdgeColor', [1 1 1]*0.7);

    %%%% Draw links
    plot(pos(1,:),pos(2,:),'Color',[0.0664, 0.422, 0.988],'LineWidth',8)
    
    %%%% Draw Joints
    plot(pos(1,1:2),pos(2,1:2),'k.','MarkerSize',60, ...
        'Color', [0.1914 0.0469 0.3242])

end