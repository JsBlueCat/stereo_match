
A=[];%x,y,z������bai
x=A(:,1);y=A(:,2);z=A(:,3);
scatter3(x,y,z)%ɢ��ͼ
figure
[X,Y,Z]=griddata(x,y,z,linspace(min(x),max(x))',linspace(min(y),max(y)),'v4');%��ֵ
pcolor(X,Y,Z);shading interp%α��ɫduͼ
figure,contourf(X,Y,Z) %�ȸ���ͼzhi
figure,surf(X,Y,Z);%��ά����dao