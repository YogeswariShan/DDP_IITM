function P = Pestcontrol(xig,yig,Ng,tsp,lst)

P=zeros(Ng);
P(Ng-yig,xig)=0.1;
t=1;
ig={[Ng-yig xig]};
sat={};

if size(lst,1)~=0
for p=1:size(lst,1)
    sat=[sat;[lst(p,1) lst(p,2)]];
    P(lst(p,1),lst(p,2))=-0.2;
end
end

while t<tsp
    
    if t<7
        for i=1:Ng
            for j=1:Ng
                for k=1:size(ig,1)
                    if isequal(ig{k},[i j]) && P(i,j)<0.8
                        P(i,j)=P(i,j)+0.05;
                        break
                    end
                end
                
            end
        end
    end
    if t<14
        for i=1:Ng
            for j=1:Ng
                if P(i,j)>0.7
                    sat=[sat;[i j]];
                end
                
            end
        end
    end
    
    if t>10
        for i=1:Ng
            for j=1:Ng
                for u=1:size(sat,1)
                    
                    if ~isequal(sat{u},[i j]) && P(i,j)>0.7
                        sat=[sat;[i j]];
                        break
                    end
                end
            end
        end
    end
    
    if t>6
        for i=1:Ng
            for j=1:Ng
                for n=1:size(ig,1)
                    if isequal(ig{n},[i j])
                        in=0;
                        for p=1:size(sat,1)
                            if isequal(sat{p},[i j])
                                in=1;
                                break
                            end
                            
                        end
                        if in==0 && P(i,j)>0
                            
                            P(i,j)=P(i,j)+0.05;
                            break
                        end
                        
                        
                        
                    end
                end
            end
        end
    end
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[1 1])
            ind=1;
            break
        end
    end
    if ind==0 && (P(1,2)>=0.4|| P(2,1)>=0.4 || P(2,2)>=0.4)
        P(1,1)=0.1;
        ig=[ig;[1 1]];
        
    end
    
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[1 Ng])
            ind=1;
            break
        end
    end
    if ind==0 && (P(1,(Ng-1))>=0.4|| P(2,(Ng-1))>=0.4 || P(2,Ng)>=0.4)
        P(1,Ng)=0.1;
        ig=[ig;[1 Ng]];
        ind=1;
    end
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[Ng 1])
            ind=1;
            break
        end
    end
    if ind==0 && (P((Ng-1),1)>=0.4|| P((Ng-1),2)>=0.4 || P(Ng,2)>=0.4)
        P(Ng,1)=0.1;
        ig=[ig;[Ng 1]];
        ind=1;
    end
    
    ind=0;
    for q=1:size(ig,1)
        if isequal(ig{q},[Ng Ng])
            ind=1;
            break
        end
    end
    if ind==0 && (P((Ng-1),(Ng-1))>=0.4|| P((Ng-1),Ng)>=0.4 || P(Ng,(Ng-1))>=0.4)
        
        P(Ng,Ng)=0.1;
        ig=[ig;[Ng Ng]];
        ind=1;
    end
    
    for i=2:Ng-1
        for j=2:Ng-1
            ind=0;
            for q=1:size(ig,1)
                if isequal(ig{q},[i j])
                    ind=1;
                    break
                end
            end
            if ind==0
                if (P((i-1),j-1)>=0.4 || P((i-1),j)>=0.4 ||P(i-1,j+1)>=0.4 || P(i,j-1)>=0.4 ||P(i,j+1)>=0.4 || P(i+1,j-1)>=0.4 || P((i+1),j)>=0.4 ||P((i+1),j+1)>=0.4)
                    P(i,j)=0.1;
                    ig=[ig;[i j]];
                    ind=1;
                    
                end
                
                
                
            end
        end
    end
    
    for i=2:Ng-1
        j=1;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P((i+1),1)>=0.4 ||P((i-1),1)>=0.4  || P((i-1),2)>=0.4 ||P(i,2)>=0.4 || P((i+1),2)>=0.4)
            P(i,1)=0.1;
            ig=[ig;[i 1]];
            ind=1;
        end
        
        j=Ng;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P(i-1,Ng)>=0.4 || P(i+1,Ng)>=0.4 || P(i-1,Ng-1)>=0.4 ||P(i,Ng-1)>=0.4 || P((i+1),Ng-1)>=0.4)
            P(i,Ng)=0.1;
            ig=[ig;[i Ng]];
            ind=1;
        end
    end
    
    for j=2:Ng-1
        i=1;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 &&(P(1,j-1)>=0.4 || P(1,j+1)>=0.4 || P(2,j-1)>=0.4 ||P(2,j)>=0.4 || P(2,j+1)>=0.4)
            
            P(1,j)=0.1;
            ig=[ig;[1 j]];
            ind=1;
        end
        
        i=Ng;
        ind=0;
        for q=1:size(ig,1)
            if isequal(ig{q},[i j])
                ind=1;
                break
            end
        end
        if ind==0 && (P(Ng,j-1)>=0.4 || P(Ng,j+1)>=0.4 || P(Ng-1,j-1)>=0.4 ||P(Ng-1,j)>=0.4 || P(Ng-1,j+1)>=0.4)
            P(Ng,j)=0.1;
            ig=[ig;[Ng j]];
            ind=1;
        end
    end
    
    t=t+1;
end

for i=1:Ng
    for j=1:Ng
        if P(i,j)<0
            P(i,j)=0;
        end
    end
end

end