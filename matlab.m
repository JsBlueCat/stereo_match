
A=[];%x,y,z轴坐标bai
x=A(:,1);y=A(:,2);z=A(:,3);
scatter3(x,y,z)%散点图
figure
[X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x))',linspace(min(y),max(y)),'v4');%插值
pcolor(X,Y,Z);shading interp%伪彩色du图
figure,contourf(X,Y,Z) %等高线图zhi
figure,surf(X,Y,Z);%三维曲面dao