import numpy as np
import pandas as pd
import re
import random
import matplotlib.pyplot as plt
import copy
##debug代码    import pdb; pdb.set_trace()

class NSGA_II(object):
        #初始化
    def __init__(self,pop,gen,pop_f):
        self.pop = pop; #种群数量
        self.gen = gen; #迭代次数
        self.pop_f=pop_f;#父代种群数量
        self.J,self.num_mac = self.data_pro() #工件信息
            # 初始调度数据
            #J[1]['a']  表示第i个工件的属性， 第一个参数为工序数  第二个参数表示释放时期 第三个参数表示交货期
            #J[1]['m'] 表示各道工序所对应的加工设备
            #J[1]['t'] 表示各道工序所对应的加工设备所需的加工时间
    def data_pro(self):
        num_mac = 15   #机器总数量
        self.num_mac=num_mac
        self.num_p = 20
        J = dict()
        #------------------------------
       # J[0]={'a':[3,0,0],'m':[[4, 3, 2],[5, 6],[1, 7, 0]],'t':[[17,  9, 17],[5 , 11],[13, 12, 20]]}
       # J[1]={'a':[3,0,0],'m':[[1, 2, 3],[0, 6],[5, 7, 4]],'t':[[12, 13, 17],[7 , 19],[9 , 12, 13]]}
       # J[2]={'a':[3,0,0],'m':[[2, 6, 1],[7, 4],[0, 3, 5]],'t':[[ 7, 15, 10],[18,  8],[14, 19,  9]]}
       # J[3]={'a':[3,0,0],'m':[[1, 2, 6],[0, 7],[4, 5, 3]],'t':[[11, 13,  6],[9 ,  6],[17,  7, 15]]}
       # J[4]={'a':[3,0,0],'m':[[5, 3, 1],[0, 4],[2, 7, 6]],'t':[[10, 17, 19],[10,  7],[13,  8,  5]]}
       # J[5]={'a':[3,0,0],'m':[[1, 0, 3],[2, 6],[7, 4, 5]],'t':[[10, 16,  9],[13,  7],[18, 15, 12]]}
       # J[6]={'a':[3,0,0],'m':[[3, 7, 5],[6, 0],[1, 2, 4]],'t':[[12, 16,  7],[12,  8],[ 6, 16,  5]]}
       # J[7]={'a':[3,0,0],'m':[[6, 4, 2],[1, 5],[3, 7, 0]],'t':[[12, 16,  7],[8 ,  9],[ 6, 16,  7]]}
        #----------------------
        #J[0]={'a':[6,0,0],'m':[[0, 3, 2],[0, 1],[1, 2, 0]],'t':[[17,  9, 17],[5 , 11],[13, 12, 20]]}
        #J[1]={'a':[3,0,0],'m':[[1, 2, 3],[0, 2],[3, 2, 1]],'t':[[12, 13, 17],[7 , 19],[9 , 12, 13]]}
        #J[2]={'a':[3,0,0],'m':[[1, 0, 3],[2, 1],[0, 3, 1]],'t':[[ 7, 15, 10],[18,  8],[14, 19,  9]]}
        #J[3]={'a':[3,0,0],'m':[[1, 2, 0],[0, 1],[2, 0, 1]],'t':[[11, 13,  6],[9 ,  6],[17,  7, 15]]}
        #------------------mk01
        #df_news = pd.read_table(r'C:\Users\hp\Desktop\论文\Fjsp benchmark date\Brandimarte_Data\Text\F_Mk01.txt',header = None)
#
        #J=dict()
        #k=0
        #df=[[] for i in range(len(df_news[0]))]
        #for i in df_news[0]:
        #    number = re.sub('\D','',i)
        #    df[k]=number
        #    J[k]={'a':[int(number[0]),0,0],'m':[[ ] for i in range(int(number[0]))],'t':[[ ] for i in range(int(number[0]))]}
        #    k+=1
        #
        #for i in range(len(df)):
        #    p=1
        #    for j in range(len(J[i]['m'])):
        #        z=int(df[i][p])
        #        J[i]['m'][j]=[int(df[i][p+(k+1)*2-1])-1  for k in range(z)]
        #        J[i]['t'][j]=[int(df[i][p+(k+1)*2])  for k in range(z)]
        #        p=p+z*2+1
        #------------MK01
        df_news = pd.read_table(r'C:\Users\hp\Desktop\论文\Fjsp benchmark date\Brandimarte_Data\Text\F_MK10.txt',header = None)

        J=dict()
        k=0
        df=[[] for i in range(len(df_news[0]))]
        for i in df_news[0]:
            number = list(map(int, i.split()))
            df[k]=number
            J[k]={'a':[int(number[0]),0,0],'m':[[ ] for i in range(int(number[0]))],'t':[[ ] for i in range(int(number[0]))]}
            k+=1
        
        for i in range(len(df)):
            p=1
            for j in range(len(J[i]['m'])):
                z=int(df[i][p])
                J[i]['m'][j]=[int(df[i][p+(k+1)*2-1])-1  for k in range(z)]
                J[i]['t'][j]=[int(df[i][p+(k+1)*2])  for k in range(z)]
                p=p+z*2+1
        #-----
        J = pd.DataFrame(J)
        return J,num_mac

    def initpop(self,J):#初始化编码 P为基于工序的编码，M为对应机器编码，N为所选设备对应的序号
        self.Sum_p = 0 #总工序数
        for i in range(np.size(J,1)): 
            self.Sum_p += J[i]['a'][0]
            pass
        P = np.zeros(self.Sum_p,dtype=np.int8)
        M = np.zeros(self.Sum_p,dtype=np.int8)
        N = np.zeros(self.Sum_p,dtype=np.int8)
        num = 0
        for i in range(np.size(J,1)): #生产工件工序加工次数表
            for j in range(J[i]['a'][0]): 
                P[num] = i
                num += 1
                pass
            pass
        np.random.shuffle(P)#随机排序
        f = dict()
        for i in range(np.size(J,1)):
            f[i]=np.where(P==i)   #选出各工件工序所在位置
            for j in range(J[i]['a'][0]):
                k = f[i][0][j] 
                s = np.size(J[i]['m'][j])
                num = np.random.randint(0,s)
                N[k] = num
                M[k] = J[i]['m'][j][num]
                pass
            pass
        return P,M,N
    
    #解码
    def decode(self,J,P,M,N): # 在关键路径的工序，进行拉动和卸载
        t_part = [[] for i in range(np.size(J,1))] #各零件加工时间
        t_mac = [[] for i in range(self.num_mac)]  #各机器使用时间
        total_n = np.size(P) #总工序数
        k_part=np.zeros(np.size(J,1),dtype=np.int8) #记录当前解码过程中工件的工序号
        k_mac=np.zeros(self.num_mac,dtype=np.int8)  #记录当前解码过程中该到工序在该设备中待加工序号
        t_span = [[] for i in range(self.num_mac)] #记录所有设备加工间隙
        t_mac_index = [[] for i in range(self.num_mac)] #各机器加工工件与序号
        
        
        
        self.IP = [[] for i in range(np.size(J,1))] #工序约束最早开始时间
        for i in range(len(t_part)):
                self.IP[i] = [0 for i in range(J[i]['a'][0])]
                
        for i in range(total_n):
            var_p = P[i] #染色体中第i个基因对应该的工件
            var_m = M[i] #染色体中第i个基因对应该的加工设备
            var_n = N[i] #染色体中第i个基因对应该设备在设备集中的序号
            
           # if var_n > len(self.J[var_p]['t'][k_part[var_p]])-1:
                 #import pdb; pdb.set_trace()
                
            pro_time = self.J[var_p]['t'][k_part[var_p]][var_n] #该道工序所需加工时间
            #确定工件工序开始时间
            if k_part[var_p] > 0:  #基于工件的约束
                start_t_p = t_part[var_p][k_part[var_p]-1][1]
            else:
                start_t_p = 0
                pass
            if k_mac[var_m] > 0:  #基于设备的约束
                start_t_m = t_mac[var_m][k_mac[var_m]-1][1]
            else:
                start_t_m = 0
                pass
            #---------------先不考虑插入因素，按顺序排程 ------------------
            
            self.IP[var_p][k_part[var_p]]=start_t_p
            
            
            start_t = np.maximum(start_t_p,start_t_m)
            stop_t = start_t + pro_time
            t_part[var_p].append([start_t,stop_t])
            t_mac[var_m].append([start_t,stop_t]) 
            t_mac[var_m] = sorted(t_mac[var_m]) #机器时间从新排序
            if k_mac[var_m] > 0:
                if start_t - t_mac[var_m][k_mac[var_m]-1][1] >0:
                    t_span[var_m].append([t_mac[var_m][k_mac[var_m]-1][1],start_t])
            else:
                if start_t - 0 >0:
                    t_span[var_m].append([0,start_t])
                    
            
            
            t_mac_index[M[i]].append([P[i],k_part[var_p]])


            k_part[var_p] = k_part[var_p]+1
            k_mac[var_m] = k_mac[var_m]+1
            #--------------
        self.k_part = k_part
        self.k_mac = k_mac
        self.t_span = t_span
        self.t_part = t_part
        self.t_mac = t_mac
        self.P = P
        self.M = M
        self.N = N
        self.t_mac_index = t_mac_index
         
        return t_part,t_mac
    

    def cal_equ_load(self,t_mac): #计算设备负荷
        t = [0 for i in range(len(t_mac))]
        for j in range(len(t_mac)):
            for i in range(len(t_mac[j])):
                t[1]+= t_mac[j][i][1]-t_mac[j][i][0]
                pass
            pass
        Wmax = np.max(t) #机器的最大负荷
        #设备圆滑指数
        #for i in range(len(t)):
        #    Sum = (Wmax - t[i])**2
        #    pass
        #t_load = (Sum/len(t_mac))**0.5
        #机器的总负荷
        t_load=sum(t)
        return t_load 
    
    def non_domination_sort(self,pro_matrix,mac_matrix): #快速非支配排序 
        C = pro_matrix.shape[0] #种群数量
        Target = 2 #优化目标数
        V = pro_matrix.shape[1]-Target #决策变量个数
        front = 0
        F = dict()
        F[0]=[]
        dom = dict() #存放支配个体的数量和个体支配的解集  dom:dominate 
        b = np.zeros([self.pop,V+Target+1]) #代替增加 分级后的 pri_matrix
        p_matrix = np.zeros([self.pop,V+Target+2])
        for i in range(C):
            dom[i]={'n':0,'p':[]} #n为可以支配i的个体数量，p为被i支配的个体集合
            for j in range(C):
                dom_good = 0 #优于对比值
                dom_equal = 0
                dom_bed = 0 #次于目标值
                for k in range(Target):
                    if pro_matrix[i,V+k]< pro_matrix[j,V+k]:
                        dom_good += 1
                    elif pro_matrix[i,V+k] == pro_matrix[j,V+k]:
                        dom_equal += 1
                    else:
                        dom_bed += 1
                        pass
                    pass
                if dom_good == 0 and dom_equal != Target:
                    dom[i]['n']+=1
                elif dom_bed == 0 and dom_equal != Target:
                    dom[i]['p'].append(j)
                    pass
                pass
            if dom[i]['n'] == 0: #个体i非支配等级排序最高，属于当前最优解集\
                b[i]=np.r_[pro_matrix[i],0]
                F[front].append(i)
                pass
            #上面的代码是为了找出等级最高的非支配解集
            #下面的代码是为了给其他个体进行分级
        while len(F[front]) != 0:
            Q = []
            for i in range(len(F[front])):#循环当前支配解集中的个体
                if len(dom[F[front][i]]['p']) != 0: #个体有可支配的解集
                    for j in range(len(dom[F[front][i]]['p'])): #遍历个体可支配的解集
                        dom[dom[F[front][i]]['p'][j]]['n'] -= 1
                        if dom[dom[F[front][i]]['p'][j]]['n'] == 0: #不再被支配
                            b[dom[F[front][i]]['p'][j]]=np.r_[pro_matrix[dom[F[front][i]]['p'][j]],front+1]
                            Q.append(dom[F[front][i]]['p'][j])
                            pass
            front += 1
            F[front] = Q
            pass
        
        sorted_b=b[np.lexsort(b.T)] #将 [pro_maxic,等级]的数组按等级顺序排序 
        m_matrix=mac_matrix[np.lexsort(b.T)]
        current_index = 0   
        for front in range(len(F)-1):
            distance = 0
            previous_index = current_index 
            y=sorted_b[sorted_b[:,V+Target]==front] #y中存放的是排序等级为front的集合矩阵
            current_index = current_index + len(F[front])
            for i in range(Target):
                sorted_b_objective =y[np.argsort(y[:,V+i])]#按照目标函数值排序
                f_max = np.max(sorted_b_objective[:,V+i])
                f_min = np.min(sorted_b_objective[:,V+i])
                #对排序后的第一个个体和最后一个个体的距离设为无穷大
                y1 = np.zeros([len(y),V+Target*2+1]) #y1 记录个体拥挤度
                y1[:,0:V+Target+1] = y
                y1[np.argmax(y[:,V+i])][V+Target+1+i]= float('inf')
                y1[np.argmin(y[:,V+i])][V+Target+1+i]= float('inf')
                for j in range(1,len(y)-1):
                    next_obj = sorted_b_objective[j+1][V+i]
                    previous_obj = sorted_b_objective[j-1][V+i]
                    if (f_max-f_min) == 0 :
                        y1[np.argsort(y[:,V+i])[j]] = float('inf')
                    else:
                        y1[np.argsort(y[:,V+i])[j]] = (next_obj - previous_obj)/(f_max - f_min)
                        pass
                    pass
                pass
            pass
            distance = np.sum( y1[:,Target+V+1:Target*2+V+1],axis=1)
            distance1 =  distance.reshape((len(distance),1))
            y = np.c_[y,distance1]
            p_matrix[previous_index:current_index,:] = y
            pass
        return  p_matrix,m_matrix
    
    def tournament_selection(self,p_matrix,m_matrix,pool_size,tour):#轮盘选择
        suiji_pool = [i for i in range(self.pop_f//2)]
        father_P = np.zeros([pool_size,m_matrix.shape[1]]) #筛选后的新父代 工序集合
        father_M = np.zeros([pool_size,m_matrix.shape[1]]) #筛选后的新父代 机器集合
        for i in range(pool_size):
            canidate = random.sample(suiji_pool,tour)
            if p_matrix[canidate[0]][self.Sum_p+2] < p_matrix[canidate[1]][self.Sum_p+2]: #等级比较
                father_P[i] = p_matrix[canidate[0]][0:self.Sum_p] 
                father_M[i] = m_matrix[canidate[0]]
            elif p_matrix[canidate[0]][self.Sum_p+2] == p_matrix[canidate[1]][self.Sum_p+2]:
                if p_matrix[canidate[0]][self.Sum_p+3] > p_matrix[canidate[1]][self.Sum_p+3]: #拥挤度比较
                    father_P[i] = p_matrix[canidate[0]][0:self.Sum_p] 
                    father_M[i] = m_matrix[canidate[0]] 
                else:
                    father_P[i] = p_matrix[canidate[1]][0:self.Sum_p] 
                    father_M[i] = m_matrix[canidate[1]]
            else:
                father_P[i] = p_matrix[canidate[1]][0:self.Sum_p] 
                father_M[i] = m_matrix[canidate[1]]
        return father_P,father_M
    
    def GA(self,father_P,father_M):
        #采用每次选择2个父代进行交叉,或选择一个父代变异，每次生成2个体或一个个体
        rran = [i for i in range(self.pop//2)]
        J_jihe=[i for i in range(self.J.shape[1])]
        index = 0
        child_P = np.zeros((self.pop-20,father_P.shape[1]))
        child_M = np.zeros((self.pop-20,father_P.shape[1]))
        crossover = 0
        mutation = 0
        while(index < self.pop-20):
            if random.random() < 0.7: #染色体交叉
                child = random.sample(rran,2)
                parent1_P = father_P[child[0],:]
                parent1_M = father_M[child[0],:]
                parent2_P = father_P[child[1],:]
                parent2_M = father_M[child[1],:]
                
                C1_P = np.zeros(len(parent1_P))
                C1_M = np.zeros(len(parent1_P))
                C2_P = np.zeros(len(parent1_P))
                C2_M = np.zeros(len(parent1_P))
               
                J1 = np.array(random.sample(J_jihe,int(len( J_jihe)))) #工件随机分成两个集合
                J2 = np.setdiff1d(J_jihe,J1)
                
                shaixuan1 = np.in1d(parent1_P,J1) #获得 父代1中与工件集合1 对应的 布尔型集合
                shaixuan2 = np.in1d(parent2_P,J2)
                
                C1_P[shaixuan1== True]= parent1_P[shaixuan1== True]
                C1_P[shaixuan1== False]= parent2_P[shaixuan2== True]
                C1_M[shaixuan1== True]= parent1_M[shaixuan1== True]
                C1_M[shaixuan1== False]= parent2_M[shaixuan2== True]
                
                C2_P[shaixuan2== True]= parent2_P[shaixuan2== True]
                C2_P[shaixuan2== False]= parent1_P[shaixuan1== True]
                C2_M[shaixuan2== True]= parent2_M[shaixuan2== True]
                C2_M[shaixuan2== False]= parent1_M[shaixuan1== True]
                #if all(C1_P==0)  == True or all(C2_P==0)  :  
                    #import pdb; pdb.set_trace()                
                crossover = 1 
                mutation = 0
            else:
                #染色体变异
                child = random.randint(0,(self.pop//2)-1)
                parent3_P = father_P[child,:]
                parent3_M = father_M[child,:]
                
                C3_P = np.zeros(len(parent3_P))
                C3_M = np.zeros(len(parent3_P))
                
                position = [i for i in range(father_P.shape[1])] #基因索引集合
                #工序变异
                x = random.sample(position,2)
                benti = np.delete(parent3_P, max(x))
                insert = parent3_P[max(x)]
                C3_P= np.insert(benti,min(x),insert)
                #机器调整
                for i in range(self.J.shape[1]):
                    C3_M[C3_P == i] =  parent3_M[parent3_P == i] 
                
                #if all(C3_P==0)  == True:
                    #import pdb; pdb.set_trace()
                
                crossover = 0
                mutation = 1

            if crossover == 1 :
                if index + 1 <= self.pop -20-1:
                    child_P[index,:] = C1_P
                    child_M[index,:] = C1_M
                    child_P[index+1,:] = C2_P
                    child_M[index+1,:] = C2_M
                else:
                    child_P[index,:] = C1_P
                    child_M[index,:] = C1_M
                index+=2
                crossover = 0
                
            elif mutation == 1:
                child_P[index,:] = C3_P
                child_M[index,:] = C3_M
                index += 1
                mutation = 0
            pass
        
        return child_P,child_M
    
    def machine_index(self,P,M):
        N = np.zeros(len(P),dtype=np.int8)
        for i in range(self.J.shape[1]):
            for j in range(len(P[P ==i])):
                k = np.where(P==i)[0][j]
                N[k] =  self.J[i]['m'][j].index(M[P == i][j])
        N = N.astype(int)
        return N
    
    def ganttChart(self,P,M,t_part):#绘制甘特图
        color = ['b','r','y','m','c','g','teal','darkolivegreen','teal','darkblue','orange','deepskyblue','honeydew','seashell','peru','pink','purple','royalblue','salmon','seagreen']
        n = [0 for i in range(len(t_part))] #记录应画第几个工件工序
        for j in range(len(M)):
            
            i = M[j]+1 #机器编号
            t = t_part[P[j]][n[P[j]]][1]-t_part[P[j]][n[P[j]]][0] #加工时间
            start_t = t_part[P[j]][n[P[j]]][0] #开始时间
            
            plt.barh(i,t,left=start_t,color=color[P[j]])
            plt.text(start_t+t/self.num_mac,i,'O%s%s'%(P[j]+1,n[P[j]]+1),color='white')
            n[P[j]]+=1
            pass
        plt.yticks(np.arange(1, self.num_mac+1, 1)) 
        plt.xlabel("Time")
        plt.ylabel('Machine')
        plt.title('Gantt')
        plt.show
        
    def loading_resource(self,P,M,N):#卸载资源
        
        c_time=0
        for l in self.t_part:
                if c_time<np.max(l):
                    c_time=np.max(l)
       # #找到关键路径
       # c_time = np.max(self.t_part) 
       # V=[] #关键路径工序集合
       # ct = c_time  #完工时间

       # while ct > 0:
       #     a1 = np.where(np.array(self.t_part)[:,:,1] == ct)[0][0] #确定工件
       #     a2 = np.where(np.array(self.t_part)[:,:,1] == ct)[1][0] #确定工序
       #     V.append([a1,a2])
       #     ct=np.array(self.t_part)[:,:,0][a1,a2] #更新下一关键工序的完工时间

        #计算最晚开始时间
        hj=[ self.J[i]['a'][0] for i in range(self.J.shape[1])] #工件工序数量

        k_part = self.k_part.copy()
        k_mac = self.k_mac.copy()

        LST = [[] for i in range(len(hj))] #最晚开始时间数值数组
        for i in range(len(hj)):
            LST[i] = [0 for i in range(hj[i])]

        LST_P = [[] for i in range(len(hj))] #工件最晚开始时间数值数组
        for i in range(len(hj)):
            LST_P[i] = [0 for i in range(hj[i])]

        LST_M = [[] for i in range(len(k_mac))] #机器最晚开始时间数值数组
        for i in range(len(k_mac)):
            LST_M[i] = [0 for i in range(k_mac[i])]

        for i in range(self.Sum_p):#各机器最晚开始时间
            Oj = self.P[self.Sum_p-i-1]#当前工件号
            Mi = self.M[self.Sum_p-i-1]#当前机器号
            Ni = self.N[self.Sum_p-i-1]#J中当前机器号位置索引
            pijh = self.J[Oj]['t'][k_part[Oj]-1][Ni]#在机器i上工件j的工序h的加工时间
            #工件约束最晚开始时间
            if k_part[Oj] == hj[Oj]: 
                LST_P1 = c_time  - pijh
            else:
                LST_P1 = LST_P[Oj][k_part[Oj]] - pijh

            #机器约束最晚开始时就
            if k_mac[Mi] == self.k_mac[Mi]:
                LST_M1 = c_time  - pijh
            else:
                LST_M1 = LST_M[Mi][k_mac[Mi]]-pijh

           #确定真正约束 
            LST1=min(LST_P1,LST_M1)

            LST_P[Oj][k_part[Oj]-1] = LST1
            LST_M[Mi][k_mac[Mi]-1] =LST1
            k_part[Oj] -=1
            k_mac[Mi] -=1
            
        
        #机器空闲时间
        t_span = [[] for i in range(len(self.t_mac))]
        for i in range(len(k_mac)):
            t_span[i] = [[] for i in range(len(self.t_mac[i]))]
            for j in range(len(self.t_mac[i])):
                t_span[i][j].append(self.t_mac[i][j][0])
                t_span[i][j].append(LST_M[i][j])

        for i in range(len(t_span)): 
            if len(t_span[i])==0: #无加工任务机器
                t_span[i].append([0,c_time])
            else: 
                t_span[i].insert(0,[0,t_span[i][0][0]])
            if len(t_span[i]) > 1: 
                p_time =self.t_mac[i][len(self.t_mac[i])-1][1]-self.t_mac[i][len(self.t_mac[i])-1][0]
                t_span[i].insert(len(t_span[i]),[t_span[i][len(t_span[i])-1][1]+p_time,c_time])
                
        #反向编码 最早开始时间EST
        import copy
        
        
        gongxu=[model.J[i]['a'][0] for i in range(self.J.shape[1])]
        EST_copy=[[] for i in range(self.J.shape[1])]
        for i in range(self.J.shape[1]):
            EST_copy[i]=[self.t_part[i][j][0] for j in range(gongxu[i])]
            
        #----   提数 
        self.EST_P11=copy.deepcopy(EST_copy)
        #------
        new_p_E = []
        
        for k in range(self.Sum_p):
            Min=10000000
            for i in EST_copy: #找到最小开始时间
                for j in i:
                    if j <Min:
                        Min = j
            for i in range(len(LST_P)): #找到最晚开始时间最小的工件和工序号
                if len((np.where(EST_copy[i]==Min))[0])==1:
                    a1=i
                    a2=(np.where(EST_copy[i]==Min))[0][0]
                    new_p_E.append(a1) #写入新工序编码
                    del(EST_copy[a1][a2]) #删除已录入编码
                    i+=10000
        new_p_E=np.array(new_p_E)
        new_m_E=np.zeros(self.Sum_p,dtype=np.int8)
        new_n_E=np.zeros(self.Sum_p,dtype=np.int8)
        for i in range(self.J.shape[1]):
            index_p1=np.in1d(new_p_E,i) #新编码机器索引
            index_p2=np.in1d(self.P,i)#旧编码机器索引
            new_m_E[index_p1==True] = self.M[index_p2==True]
            new_n_E[index_p1==True] = self.N[index_p2==True]
            

   
        #反向编码 最晚开始时间LST

        import copy
        LST_copy = copy.deepcopy(LST_P)
        new_p_L = []

        for k in range(self.Sum_p):
            Min=10000000
            for i in LST_copy: #找到最小开始时间
                for j in i:
                    if j <Min:
                        Min = j
            for i in range(len(LST_P)): #找到最晚开始时间最小的工件和工序号
                if len((np.where(LST_copy[i]==Min))[0])==1:
                    a1=i
                    a2=(np.where(LST_copy[i]==Min))[0][0]
                    new_p_L.append(a1) #写入新工序编码
                    del(LST_copy[a1][a2]) #删除已录入编码
                    i+=10000
        new_p_L=np.array(new_p_L)
        new_m_L=np.zeros(self.Sum_p,dtype=np.int8)
        new_n_L=np.zeros(self.Sum_p,dtype=np.int8)
        for i in range(self.J.shape[1]):
            index_p1=np.in1d(new_p_L,i) #新编码机器索引
            index_p2=np.in1d(self.P,i)#旧编码机器索引
            new_m_L[index_p1==True] = self.M[index_p2==True]
            new_n_L[index_p1==True] = self.N[index_p2==True] 

   
            #-提些数据写论文
        self.LST_P11 =LST_P
        self.t_part11 =self.t_part
        self.new_p_E = new_p_E
        self.new_m_E =new_m_E
        self.new_p_L=new_p_L
        self.new_m_L=new_m_L
        self.p11 = P
        self.m11=M
        
        
        
        
        #关键工序，浮动时间为0
        
        ft=[[] for i in range(model.num_p)]
        V=[]
        for i in range(model.num_p):
            ft[i]=np.array(model.LST_P11[i])-np.array(model.EST_P11[i])
            V_index=np.where(ft[i]==0)
            for j in range(len(V_index[0])):
                V.append([i,V_index[0][j]])
        
                
        #确定插入位置 和插入
        q = 0 #当前优化关键工序在关键路径的位置
        while q < len(V):
            
            Oj = V[q][0]#工件序号
            h = V[q][1]#工序序号
            hj = self.J[Oj]['a'][0]-1 #工序总数
            O_mac = self.J[Oj]['m'][h] #可选机台集合
            N_mac = len(self.J[Oj]['t'][h]) #可选机台数量
            T_mac = self.J[Oj]['t'][h] # 可选机台加工时间
            EST_P = self.IP[Oj][h]

            e = 0 #机器集中序号，N
            i = 0
            a1=0  #插入机器序号
            a2=0  #插入机器位置
            a3=0 #机器集中序号,在下一步中被赋值

            N_idel = 0 #机器空闲时间段序号
            keyong = False
            e=0
            while e < N_mac and keyong == False: 
                k=0
                if h < hj:
                    while k < len(t_span[O_mac[e]]) and  keyong == False:
                        if  EST_P < LST_P[Oj][h+1]- T_mac[e] and t_span[O_mac[e]][k][0] < LST_P[Oj][h+1]- T_mac[e] and t_span[O_mac[e]][k][1]- T_mac[e] >EST_P and t_span[O_mac[e]][k][1]-T_mac[e]> t_span[O_mac[e]][k][0]:
                            a1=O_mac[e] 
                            a2=k
                            a3=e
                            keyong = True
                            pass
                        k+=1
                            #import pdb; pdb.set_trace()

                else:
                     while k < len(t_span[O_mac[e]]) and  keyong == False:
                        if t_span[O_mac[e]][k][1]- T_mac[e] >EST_P and t_span[O_mac[e]][k][1]-T_mac[e]> t_span[O_mac[e]][k][0]:
                            a1=O_mac[e]
                            a2=k
                            a3=e
                            keyong = True
                            pass
                        k+=1
                e+=1           
                
                
            if keyong == True:
                    
                if len( self.t_mac_index[a1])==0 : #如果机器上没有工序或机器空闲时间段第一位
                    x1=np.where(new_p_L==Oj)[0][h] #只改变机器染色体
                    x2=x1
                    benti_p = np.delete(new_p_L, x1)
                    benti_m = np.delete(new_m_L, x1)
                    benti_n = np.delete(new_n_L, x1)
                    insert_p = new_p_L[x1]
                    insert_m = O_mac[a3]
                    insert_n = a3
                    un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                    un_m = np.insert(benti_m,x1,insert_m)
                    un_n = np.insert(benti_n,x1,insert_n)

                elif a2==0:  #机器空闲时间段第一位，都在机器第一个加工机器之前
                    xuhao = self.t_mac_index[a1][0]
                    x1=np.where(new_p_L==Oj)[0][h] #卸载工序
                    x2=np.where(new_p_L==xuhao[0])[0][xuhao[1]] #插入位置
                    if x1 <x2:
                        benti_p = np.delete(new_p_L, x1)
                        benti_m = np.delete(new_m_L, x1)
                        benti_n = np.delete(new_n_L, x1)
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3
                        un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                        un_m = np.insert(benti_m,x1,insert_m)
                        un_n = np.insert(benti_n,x1,insert_n)
                    else:#移动到x2之前
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3                
                        benti_p = np.insert(new_p_L,x2,insert_p)# 先插入
                        benti_m = np.insert(new_m_L,x2,insert_m)
                        benti_n = np.insert(new_n_L,x2,insert_n) 
                        un_p = np.delete(benti_p, x1+1)#大序号插向小序号，删除本身位置+1
                        un_m = np.delete(benti_m, x1+1)
                        un_n= np.delete(benti_n, x1+1)     

                elif a2 == len(t_span[a1])-1:#机器空闲时间段最后一段，插入机器最后一个工序之后
                    xuhao = self.t_mac_index[a1][a2-2]
                    x1=np.where(new_p_L==Oj)[0][h] #卸载工序
                    x2=np.where(new_p_L==xuhao[0])[0][xuhao[1]]+1 #插入位置
                    if x1>x2 :
                        benti_p = np.delete(new_p_L, x1)
                        benti_m = np.delete(new_m_L, x1)
                        benti_n = np.delete(new_n_L, x1)
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3
                        un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                        un_m = np.insert(benti_m,x1,insert_m)
                        un_n = np.insert(benti_n,x1,insert_n)
                    else: 
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3                
                        benti_p = np.insert(new_p_L,x2,insert_p)# 先插入
                        benti_m = np.insert(new_m_L,x2,insert_m)
                        benti_n = np.insert(new_n_L,x2,insert_n)
                        un_p = np.delete(benti_p, x1) #小序号插向大序号，删除本体位置
                        un_m = np.delete(benti_m, x1)
                        un_n = np.delete(benti_n, x1)

                else: #其他机器空闲时间段，插入对应机器之前
                    xuhao = self.t_mac_index[a1][a2-1]
                    x1=np.where(new_p_L==Oj)[0][h] #卸载工序
                    x2=np.where(new_p_L==xuhao[0])[0][xuhao[1]] #插入位置
                    
                    #
                    exist_front=False
                    exist_front=False
                    if h-1 >= 0:
                        x4 =np.where(new_p_L==Oj)[0][h-1] #前道工序序号
                        exist_front=True
                        
                        
                    if h+1 < len(np.where(new_p_L==Oj)[0])-1:# 移动min(后道工序位置，插入机器位置)前
                        x3=np.where(new_p_L==Oj)[0][h+1] #后道工序序号
                        x2=min(x2,x3)
                        exist_front=True
                        pass
                    
                    if xuhao[0]==Oj:#如果为插入位置为同道工序，只改变机器号。
                        benti_p = np.delete(new_p_L, x1)
                        benti_m = np.delete(new_m_L, x1)
                        benti_n = np.delete(new_n_L, x1)
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3
                        un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                        un_m = np.insert(benti_m,x1,insert_m)
                        un_n = np.insert(benti_n,x1,insert_n)                    
                    
                    else:
                        insert_p = new_p_L[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3                
                        benti_p = np.insert(new_p_L,x2,insert_p)# 先插入
                        benti_m = np.insert(new_m_L,x2,insert_m)
                        benti_n = np.insert(new_n_L,x2,insert_n)   
                        if x1<x2: #小序号插向大序号，删除本体位置
                            un_p = np.delete(benti_p, x1)
                            un_m = np.delete(benti_m, x1)
                            un_n = np.delete(benti_n, x1)
                        else:#大序号插向小序号，删除本身位置+1
                            un_p = np.delete(benti_p, x1+1)
                            un_m = np.delete(benti_m, x1+1)
                            un_n= np.delete(benti_n, x1+1)
                    #判断是调整是否合理
                    hege=True
                    self.hege=hege
                    a=np.where(un_p==Oj)[0]
                    for i in range(len(a)):
                        if len(np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0])==0:
                            hege=False
                            break
                        else:
                            true_n=np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0][0]
                            if true_n != un_n[a[i]]:
                                hege=False
                                break 
                    if hege==False and h+1 <= hj: #将该工序原本的后一到工序 移到当前位置之后
                        
                        x1=np.where(new_p_L==Oj)[0][h+1]-1 #卸载工序
                        insert_p = un_p[x1]
                        insert_m = un_m[x1]
                        insert_n = un_n[x1]                
                        benti_p = np.insert(un_p,x2,insert_p)# 先插入
                        benti_m = np.insert(un_m,x2,insert_m)
                        benti_n = np.insert(un_n,x2,insert_n)   
                        if x1<x2: #小序号插向大序号，删除本体位置
                            un_p = np.delete(benti_p, x1)
                            un_m = np.delete(benti_m, x1)
                            un_n = np.delete(benti_n, x1)
                        else:#大序号插向小序号，删除本身位置+1
                            un_p = np.delete(benti_p, x1+1)
                            un_m = np.delete(benti_m, x1+1)
                            un_n= np.delete(benti_n, x1+1)
                                
                #debug 插入代码出现错误
                #k_part=np.zeros(np.size(self.J,1),dtype=np.int8)
                #for pp in range(12):
                #
                #    var_p=un_p[pp]
                #    var_n=un_n[pp]
                #    if var_n > len(self.J[var_p]['t'][k_part[var_p]])-1  or :
                #        import pdb; pdb.set_trace()
                #        
                #    
                #    
                #    self.new_p=new_p
                #    self.new_m=new_m
                #    self.new_n=new_n
                #    self.t_part=self.t_part
                #    self.un_p=un_p
                #    self.un_m=un_m
                #    self.un_n=un_n
                #    self.x1=x1
                #    self.x2=x2
                #    N = self.machine_index(un_p,un_m)
                #    return un_p,un_m,un_n                               
                #    k_part[var_p]+=1    
                #
                #return un_p,un_m,un_n
                
                #判断是调整是否合理
                hege=True
                self.hege=hege
                a=np.where(un_p==Oj)[0]
                for i in range(len(a)):
                    if len(np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0])==0:
                        hege=False
                        break
                    else:
                        true_n=np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0][0]
                        if true_n != un_n[a[i]]:
                            hege=False
                            break  
                
                if  hege==False: #之前调整不合理
                   # print("不合格")
                    #------出现问题 重新从前插入
                    if len( self.t_mac_index[a1])==0 : #如果机器上没有工序或机器空闲时间段第一位
                        #print('无机器')
                        x1=np.where(new_p_E==Oj)[0][h] #只改变机器染色体
                        x2=x1
                        benti_p = np.delete(new_p_E, x1)
                        benti_m = np.delete(new_m_E, x1)
                        benti_n = np.delete(new_n_E, x1)
                        insert_p = new_p_E[x1]
                        insert_m = O_mac[a3]
                        insert_n = a3
                        un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                        un_m = np.insert(benti_m,x1,insert_m)
                        un_n = np.insert(benti_n,x1,insert_n)
                    elif a2==0:  #机器空闲时间段第一位，都在机器第一个加工机器之前1
                       # print('第一段')
                        xuhao = self.t_mac_index[a1][0]
                        x1=np.where(new_p_E==Oj)[0][h] #卸载工序
                        x2=np.where(new_p_E==xuhao[0])[0][xuhao[1]] #插入位置
                        if x1 <x2:
                            x2=x1
                            benti_p = np.delete(new_p_E, x1)
                            benti_m = np.delete(new_m_E, x1)
                            benti_n = np.delete(new_n_E, x1)
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3
                            un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                            un_m = np.insert(benti_m,x1,insert_m)
                            un_n = np.insert(benti_n,x1,insert_n)
                        else:#移动到x2之前
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3                
                            benti_p = np.insert(new_p_E,x2,insert_p)# 先插入
                            benti_m = np.insert(new_m_E,x2,insert_m)
                            benti_n = np.insert(new_n_E,x2,insert_n) 
                            un_p = np.delete(benti_p, x1+1)#大序号插向小序号，删除本身位置+1
                            un_m = np.delete(benti_m, x1+1)
                            un_n= np.delete(benti_n, x1+1)  
                    elif a2 == len(t_span[a1])-1:#机器空闲时间段最后一段，插入机器最后一个工序之后
                        #print('最后一段')
                        xuhao = self.t_mac_index[a1][a2-2]
                        x1=np.where(new_p_E==Oj)[0][h] #卸载工序
                        x2=np.where(new_p_E==xuhao[0])[0][xuhao[1]]+1 #插入位置
                        if x1>x2 :
                            x2=x1
                            benti_p = np.delete(new_p_E, x1)
                            benti_m = np.delete(new_m_E, x1)
                            benti_n = np.delete(new_n_E, x1)
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3
                            un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                            un_m = np.insert(benti_m,x1,insert_m)
                            un_n = np.insert(benti_n,x1,insert_n)
                        else: 
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3                
                            benti_p = np.insert(new_p_E,x2,insert_p)# 先插入
                            benti_m = np.insert(new_m_E,x2,insert_m)
                            benti_n = np.insert(new_n_E,x2,insert_n)
                            un_p = np.delete(benti_p, x1) #小序号插向大序号，删除本体位置
                            un_m = np.delete(benti_m, x1)
                            un_n = np.delete(benti_n, x1)
                    else: #其他机器空闲时间段，插入对应机器之前
                        xuhao = self.t_mac_index[a1][a2-1]
                        x1=np.where(new_p_E==Oj)[0][h] #卸载工序
                        x2=np.where(new_p_E==xuhao[0])[0][xuhao[1]] #插入位置
                        
                        #
                        if h-1 >= 0:#移动max(前道工序位置，插入机器位置)前
                            x4 =np.where(new_p_E==Oj)[0][h-1] #前道工序序号
                            x2=max(x2,x4) 
                            exist_front=True
                            
                            
                        if h+1 < len(np.where(new_p_E==Oj)[0])-1:# 移动min(后道工序位置，插入机器位置)前
                            x3=np.where(new_p_E==Oj)[0][h+1] #后道工序序号
                            #x2=min(x2,x3)
                            exist_front=True
                            pass
                        
                        if xuhao[0]==Oj:#如果为插入位置为同道工序，只改变机器号。
                                                        #--------------------------
                           # print('haha1')
                            
                            benti_p = np.delete(new_p_E, x1)
                            benti_m = np.delete(new_m_E, x1)
                            benti_n = np.delete(new_n_E, x1)
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3
                            un_p = np.insert(benti_p,x1,insert_p)#完成卸载后工序
                            un_m = np.insert(benti_m,x1,insert_m)
                            un_n = np.insert(benti_n,x1,insert_n)
                        
                        else:
                            #--------------------------
                            #print('haha')
                            insert_p = new_p_E[x1]
                            insert_m = O_mac[a3]
                            insert_n = a3                
                            benti_p = np.insert(new_p_E,x2+1,insert_p)# 先插入
                            benti_m = np.insert(new_m_E,x2+1,insert_m)
                            benti_n = np.insert(new_n_E,x2+1,insert_n)   
                            if x1<x2: #小序号插向大序号，删除本体位置
                                un_p = np.delete(benti_p, x1)
                                un_m = np.delete(benti_m, x1)
                                un_n = np.delete(benti_n, x1)
                            else:#大序号插向小序号，删除本身位置+1
                                un_p = np.delete(benti_p, x1+1)
                                un_m = np.delete(benti_m, x1+1)
                                un_n= np.delete(benti_n, x1+1) 
                
                if hege == False:
                    #判断是调整是否合理
                    hege=True
                    
                    a=np.where(un_p==Oj)[0]
                    for i in range(len(a)):
                        if len(np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0])==0:
                            hege=False
                            break
                        else:
                            true_n=np.where(self.J[Oj]['m'][i]==un_m[a[i]])[0][0]
                            if true_n != un_n[a[i]]:
                                hege=False
                                break
                    if hege==False:
                        self.hege=hege
                        #import pdb; pdb.set_trace()
                        print('失败')
                        return P,M,N
                        
                
                return un_p,un_m,un_n                 
                break
            q+=1
                                       
        
        return P,M,N  
    
    def main(self):
        P,M,N = self.initpop(self.J)
        pro_matrix=np.zeros((self.pop_f,self.Sum_p+2)) #包含工序及目标函数值的决策矩阵
        mac_matrix=np.zeros((self.pop_f,self.Sum_p)) #包含设备染色体信息的决策矩阵
        for i in range(self.pop_f):
            P,M,N = self.initpop(self.J)
            t_part,t_mac = self.decode(self.J,P,M,N)
            
            #最大完工时间 c_time=cal_comp_time(t_part);
            c_time=0
            for l in t_part:
                    if c_time<np.max(l):
                        c_time=np.max(l)
                        
            self.c_time=c_time
            # d_time = cal_tardiness(self.J,t_part) #总延期时常(total tardiness) d_time=cal_def_time(J,t_part);
            
            t_load = self.cal_equ_load(t_part) #设备负荷平滑指数
            
            
            pro_matrix[i]=np.r_[P,[c_time,t_load]]
            mac_matrix[i] = M
            pass
        self.sl_c=[]
        self.sl_w=[]
        for i  in range(self.gen):
            
            pool = self.pop//2
            tour = 2
            p_matrix,m_matrix = self.non_domination_sort(pro_matrix,mac_matrix)#进行非支配快速排序
            pro_matrix=np.zeros((self.pop,self.Sum_p+2)) 
            #收敛速度
            self.sl_c.append(min( p_matrix[:,self.Sum_p]))
            self.sl_w.append(min( p_matrix[:,self.Sum_p+1]))
            #----------------
            mac_matrix=np.zeros((self.pop,self.Sum_p)) 
            father_P,father_M = self.tournament_selection(p_matrix,m_matrix,pool,tour) #精英选择适合繁殖的父代
            
            child_P,child_M = self.GA(father_P,father_M) #变异生成子代种群
            
            
            minc=p_matrix[np.argsort(p_matrix[:,self.Sum_p])]  
            minm=m_matrix[np.argsort(p_matrix[:,self.Sum_p])]
            new_P=np.concatenate((minc[0:20,0:self.Sum_p],child_P),axis=0)  #精英父代和子代合并
            new_M=np.concatenate((minm[0:20,0:self.Sum_p],child_M),axis=0)
    
            for j in range(self.pop_f):
                #print(j)
                P = new_P[j,:].astype(int)
                M = new_M[j,:].astype(int)
                N = self.machine_index(P,M)
                t_part,t_mac = self.decode(self.J,P,M,N)
                
                void_times =0 #无效次数
                while void_times <= self.num_mac :
                    c_time=0
                    for l in t_part:
                            if c_time<np.max(l):
                                c_time=np.max(l)
                    resoult = c_time
                    #print(j,resoult)
                    P,M,N = self.loading_resource(P,M,N)
                    t_part,t_mac = self.decode(self.J,P,M,N)
                    c_time=0#Z最大完工时间
                    for l in t_part:
                            if c_time<np.max(l):
                                c_time=np.max(l)
                            
                    if resoult == c_time:
                        void_times +=1
                    elif resoult < c_time:
                        break
                    else :
                        void_times =0
                    if self.hege==False:
                         break
                    
                     
                # d_time = cal_tardiness(self.J,t_part) #总延期时常(total tardiness) d_time=cal_def_time(J,t_part);
                t_load = self.cal_equ_load(t_part) #设备负荷平滑指数
                pro_matrix[j] = np.r_[P,[c_time,t_load]]
                mac_matrix[j] = M

                pass

           #去除重复染色体
            p_unqie=np.array(list(set([tuple(t) for t in pro_matrix])))
            m_unqie=[]
            np.where(pro_matrix==p_unqie[0])[0]
            for j in range(p_unqie.shape[0]):
                k=0
                for i in pro_matrix==p_unqie[j]:
                    if all(i) == True:
                        m_unqie.append(mac_matrix[k])
                        break
                    k+=1
            m_unqie=np.array(m_unqie)
            
            
            self.p_unqie=p_unqie
             #补充种群数量
            start_num =p_unqie.shape[0]#补充数量开始值
            for i in range(start_num,self.pop_f):
                P,M,N = self.initpop(self.J)
                t_part,t_mac = self.decode(self.J,P,M,N)
                
                c_time=0
                for l in t_part:
                        if c_time<np.max(l):
                            c_time=np.max(l) #最大完工时间 c_time=cal_comp_time(t_part);
                # d_time = cal_tardiness(self.J,t_part) #总延期时常(total tardiness) d_time=cal_def_time(J,t_part);
                t_load = self.cal_equ_load(t_part) #设备负荷平滑指数
                p_unqie=np.vstack([p_unqie,np.r_[P,[c_time,t_load]]])
                m_unqie=np.vstack([m_unqie,M])
                pass
            pro_matrix=p_unqie
            mac_matrix=m_unqie
            self.pro_matrix = pro_matrix
            self.mac_matrix = mac_matrix
            
        p_matrix,m_matrix = self.non_domination_sort(pro_matrix,mac_matrix)
        Target_p = p_matrix[p_matrix[:,self.Sum_p+2] == 0]
        Target_m = m_matrix[p_matrix[:,self.Sum_p+2] == 0]
        #--------------
        self.Target_p=Target_p
        self.min_2=Target_p[:,self.Sum_p:self.Sum_p+2]
        self.Target_m=Target_m 
        
        
        
        min_sum=Target_p[:,self.Sum_p]
        self.min_sum =  min(min_sum)
        
        best_p_index = np.where(min_sum==min(min_sum))[0][0] 
        self.best_p = Target_p[best_p_index]  #选择 目标值相加最小的集合
        self.best_m = Target_m[best_p_index]
        P = self.best_p[0:self.Sum_p].astype(int)
        M = self.best_m[0:self.Sum_p].astype(int)
        N = self.machine_index(P,M)
        t_part,t_mac = self.decode(self.J,P,M,N)
        self.ganttChart(P,M,t_part)

