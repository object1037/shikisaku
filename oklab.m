[r,g,b] = meshgrid(linspace(0,1,50));
rgb = [r(:), g(:), b(:)];

l = 0.4122214708 * rgb(:,1) + 0.5363325363 * rgb(:,2) + 0.0514459929 * rgb(:,3);
m = 0.2119034982 * rgb(:,1) + 0.6806995451 * rgb(:,2) + 0.1073969566 * rgb(:,3);
s = 0.0883024619 * rgb(:,1) + 0.2817188376 * rgb(:,2) + 0.6299787005 * rgb(:,3);
l_ = nthroot(l, 3);
m_ = nthroot(m, 3);
s_ = nthroot(s, 3);
L = 0.2104542553*l_ + 0.7936177850*m_ - 0.0040720468*s_;
a = 1.9779984951*l_ - 2.4285922050*m_ + 0.4505937099*s_;
b = 0.0259040371*l_ + 0.7827717662*m_ - 0.8086757660*s_;
L = L*100;

% fig = figure;

k = boundary(a,b,L);
trisurf(k,a,b,L,'FaceColor','interp',...
    'FaceVertexCData',rgb,'EdgeColor','none')
xlabel('a*')
ylabel('b*')
zlabel('L*')
axis([-0.4 0.4 -0.4 0.4 0 100])
axis vis3d
title('sRGB gamut surface in OKLab space','Position',[0,0,150])

filename = 'oklab.gif';
for i = 1:360
   camorbit(2,0,'data',[0 0 1])
   drawnow
   % frame = getframe(fig);
   % im = frame2im(frame);
   % [imind, cm] = rgb2ind(im,512);
   % if i == 1
   %     imwrite(imind,cm,filename,'gif','LoopCount',inf,'DelayTime',0.1);
   % else
   %     imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);
   % end
end
