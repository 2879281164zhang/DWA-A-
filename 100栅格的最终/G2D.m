function D=G2D(G) 
l=size(G,1); 
D=zeros(l*l,l*l); 
for i=1:l 
    for j=1:l 
         if G(i,j)==0 
            for m=1:l 
                for n=1:l 
                    if G(m,n)==0 
                        im=abs(i-m);jn=abs(j-n); 
                        if im+jn==1||(im==1&&jn==1) 
                        D((i-1)*l+j,(m-1)*l+n)=(im+jn)^0.5; 
                        end
                    end
                end
            end
         else
            if i==1
                if j==1
                    D(i+1,i+l*i)=inf;
                    D(i+l*i,i+1)=inf;
                else
                    if j==l%若障碍物为第一行的最后一个
                        D(l-1,l+l*i)=inf;
                        D(l+l*i,l-1)=inf;
                    else                %若障碍物为第一行的其他
                        D(j-1,j+i*l)=inf;
                        D(j+i*l,j-1)=inf;
                        D(j+1,j+i*l)=inf;
                        D(j+i*l,j+1)=inf;
        
                    end
                end
            end
         

            if i==l               %若障碍物在最后一行
            if j==1               %若障碍物为最后一行的第一个
                D(j+l*(i-2),j+l*(i-1)+1)=inf;
                D(j+l*(i-1)+1,j+l*(i-2))=inf;
            else
            if j==l            %若障碍物为最后一行的最后一个
                D(l*l-1,(l-1)*l)=inf;
                D((l-1)*l,l*l-1)=inf;
            else                   %若障碍物为最后一行的其他
                D((i-2)*l+j,(i-1)*l+j-1)=inf;
                D((i-1)*l+j-1,(i-2)*l+j)=inf;
                D((i-2)*l+j,(i-1)*l+j+1)=inf;
                D((i-1)*l+j+1,(i-2)*l+j)=inf;
             end
             end
            end
            if j==1               
            if i~=1&&i~=l      %若障碍物在第一列非边缘位置 
                D(j+(i-2)*l,j+1+(i-1)*l)=inf;
                D(j+1+(i-1)*l,j+(i-2)*l)=inf;
                D(j+1+(i-1)*l,j+i*l)=inf;
                D(j+i*l,j+1+(i-1)*l)=inf;
             end
            end
            if j==l
            if i~=1&&i~=l        %若障碍物在最后一列非边缘位置 
               D((i+1)*l,i*l-1)=inf;
               D(i*l-1,(i+1)*l)=inf;
               D(i*l-1,(i-1)*l)=inf;
               D((i-1)*l,i*l-1)=inf;
            end
            end
            if(i~=1&&i~=l&&j~=1&&j~=l)   %若障碍物在非边缘位置
               D(j+(i-1)*l-1,j+i*l)=inf;
               D(j+i*l,j+(i-1)*l-1)=inf;
               D(j+i*l,j+(i-1)*l+1)=inf;
               D(j+(i-1)*l+1,j+i*l)=inf;
               D(j+(i-1)*l-1,j+(i-2)*l)=inf;
               D(j+(i-2)*l,j+(i-1)*l-1)=inf;
               D(j+(i-2)*l,j+(i-1)*l+1)=inf;
               D(j+(i-1)*l+1,j+(i-2)*l)=inf;
            end
       end
     end
end
end