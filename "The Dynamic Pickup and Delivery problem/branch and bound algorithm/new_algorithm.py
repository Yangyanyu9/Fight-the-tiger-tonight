import gc
import copy
import numpy as np
from algorithm.code_stack import check_sequence,check_capacity,record_time

from src.common.node import Node
from src.common.route import Map
from src.conf.configs import Configs
from src.utils.input_utils import get_factory_info, get_route_map
from src.utils.json_tools import convert_nodes_to_json
from src.utils.json_tools import get_vehicle_instance_dict, get_order_item_dict
from src.utils.json_tools import read_json_from_file, write_json_to_file
from src.utils.logging_engine import logger


# 合并相邻重复节点 Combine adjacent-duplicated nodes.

def __combine_duplicated_nodes(nodes):
    n = 0
    while n < len(nodes)-1:
        if nodes[n].id == nodes[n+1].id:
            nodes[n].pickup_items.extend(nodes[n+1].pickup_items)
            nodes[n].delivery_items.extend(nodes.pop(n+1).delivery_items)
        else:
            n += 1
    return nodes
#合并同一订单的item  
"""          
def __combine_same_order(id_to_unallocated_order_item:dict):
    x_id_list=[]
    x_list=[]
    pack_id_to_unallocated_order_item={}
    pick_id_to_same_order = {}
    copy_id_to_unallocated_order_item=copy.deepcopy(id_to_unallocated_order_item)
    for x_id,x in copy_id_to_unallocated_order_item.items():
        x_id_list.append(x_id)
        x_list.append(x)
    i = 0
    pick_id = []
    while i <= len(x_list)-1 :
        pick_id.append(x_id_list[i])

        if  i < len(x_list)-1 and x_id_list[i][:-2]==x_id_list[i+1][:-2] :
            sum_demand = x_list[i].demand+x_list[i+1].demand
            #demand小于15，同一order分到一组
            if sum_demand <=15:
                x_list[i].demand += x_list[i+1].demand
                x_list[i].load_time += x_list[i+1].load_time
                x_list[i].unload_time +=x_list[i+1].unload_time
                pick_id.append(x_id_list[i+1])
                x_list.pop(i+1)
                x_id_list.pop(i+1)
            else:
                pick_id_to_same_order[x_id_list[i]]=list(set(pick_id))
                pick_id = []
                pack_id_to_unallocated_order_item[x_id_list[i]] = x_list[i]
                i+=1
        else:
            pick_id_to_same_order[x_id_list[i]]=list(set(pick_id))
            pick_id = []
            pack_id_to_unallocated_order_item[x_id_list[i]] = x_list[i]
            i+=1
    return  pack_id_to_unallocated_order_item,pick_id_to_same_order
"""
def __combine_same_order(id_to_unallocated_order_item:dict):   
    x_id_list=[]
    x_list=[]
    pack_id_to_unallocated_order_item={}
    pick_id_to_same_order = {}
    #copy_id_to_unallocated_order_item=copy.deepcopy(id_to_unallocated_order_item)
    for x_id,x in id_to_unallocated_order_item.items():
        x_id_list.append(x_id)
        x_list.append(x)
    i = 0
    pick_id = []
    while i <= len(x_list)-1 :
        pick_id.append(x_id_list[i])

        #if  i < len(x_list)-1 and x_id_list[i][:-2]==x_id_list[i+1][:-2] :
        if i < len(x_list)-1 and x_list[i].pickup_factory_id==x_list[i+1].pickup_factory_id and  x_list[i].delivery_factory_id == x_list[i+1].delivery_factory_id:
            sum_demand = x_list[i].demand+x_list[i+1].demand
            #demand小于15，同一order分到一组
            if sum_demand <=15:
                x_list[i].demand += x_list[i+1].demand
                x_list[i].load_time += x_list[i+1].load_time
                x_list[i].unload_time +=x_list[i+1].unload_time
                pick_id.append(x_id_list[i+1])
                x_list.pop(i+1)
                x_id_list.pop(i+1)
            else:
                pick_id_to_same_order[x_id_list[i]]=list(set(pick_id))
                pick_id = []
                pack_id_to_unallocated_order_item[x_id_list[i]] = x_list[i]
                i+=1
        else:
            pick_id_to_same_order[x_id_list[i]]=list(set(pick_id))
            pick_id = []
            pack_id_to_unallocated_order_item[x_id_list[i]] = x_list[i]
            i+=1
    return  pack_id_to_unallocated_order_item,pick_id_to_same_order
# good_route中pick_item替换逻辑代码
def __replace_item(good_route,pick_id_to_same_order,pre_matching_item_ids):
    items_residue_times ={x:0 for x  in good_route}
    for x in good_route:
        items_residue_times[x]+=1
    copy_good_route = copy.copy(good_route)
    index = 0
    for i in range(len(copy_good_route)) :
        if copy_good_route[i] in pick_id_to_same_order and copy_good_route[i] not in pre_matching_item_ids:
            if items_residue_times[copy_good_route[i]] == 2:
                items_residue_times[copy_good_route[i]]-=1
                good_route.pop(index+i)
                for n in range(len(pick_id_to_same_order[copy_good_route[i]])):
                    good_route.insert(index+i+n,pick_id_to_same_order[copy_good_route[i]][n])
                index +=  n
            elif items_residue_times[copy_good_route[i]] == 1:
                items_residue_times[copy_good_route[i]]-=1
                good_route.pop(index+i)
                for n in range(len(pick_id_to_same_order[copy_good_route[i]])):
                    good_route.insert(index+i+n,pick_id_to_same_order[copy_good_route[i]][len(pick_id_to_same_order[copy_good_route[i]])-1-n])
                index +=  n
    return good_route

#返回计划路线
def vehicle_planned_route(vehicle_unallocated_dict:dict,code:list,id_to_factory: dict):

    planned_route = []
    good_route_item = []
    good_route_item.extend([vehicle_unallocated_dict[x] for x in code ])
    items_residue_times ={x:0 for x  in code}
    for x in code:
        items_residue_times[x]+=1
    for x in code:
        item = vehicle_unallocated_dict[x]
        if items_residue_times[x] == 2:
            items_residue_times[x] -= 1
            #pick up node
            pickup_factory_id = item.pickup_factory_id
            pickup_factory = id_to_factory.get(pickup_factory_id)
            node = Node(pickup_factory.id, pickup_factory.lng, pickup_factory.lat, [item], [])
        elif items_residue_times[x] == 1:
            items_residue_times[x] -= 1
            delivery_factory_id = item.delivery_factory_id
            delivery_factory = id_to_factory.get(delivery_factory_id)
            node = Node(delivery_factory.id, delivery_factory.lng, delivery_factory.lat, [], [item])
            #delivery node
        planned_route.append(node)
    return planned_route
 #def    
def dispatch_orders_to_vehicle(id_to_factory, id_to_unallocated_order_item, id_to_ongoing_order_item, id_to_vehicle,route_map):
     #创建车辆 未分配的item_id信息字典（包括所有已携带订单，因为未装载的可以重新分配顺序） eg ；{ '2300460049-1':<src.common.order.OrderItem object at 0x00000173639C2520>}
    vehicles_unallocated_dict = { vehicle_id : {} for vehicle_id,vehicle in id_to_vehicle.items() }
    #车辆已经carrying货物（包括实际已装和假设在目的地装）
    vehicles_loaded_items={}
    pre_matching_item_ids = []
    pro_code_type = { vehicle_id:0  for vehicle_id,vehicle in id_to_vehicle.items()}
    for vehicle_id,vehicle in id_to_vehicle.items():
        #路径顺序编码(根据item_id,order_id会重复)
            #分2种情况，case1：有目的地；case3:无目的地
        if vehicle.destination is not None:
            #应分配route的items（需卸载和装载）,当前携带items 除去即将卸载items,装上准备装的item
            pro_code_type[vehicle_id] = 1
            allocate_items_list = [item for item in vehicle.carrying_items.items if item not in vehicle.destination.delivery_items ]
            pickup_items = vehicle.destination.pickup_items #取货单
            pre_matching_item_ids.extend([item.id for item in pickup_items])#在目的地准备装货订单
            allocate_items_list.extend([item for item in pickup_items])
            
            #case2:将携带item创建（有无皆可）
        elif  vehicle.destination is  None:
            pro_code_type[vehicle_id] =2
            allocate_items_list = [item for item in vehicle.carrying_items.items  ]
        
        #车辆假设已经装载 itme 的id（实际已装载和即将在目的地装载）
        vehicles_loaded_items[vehicle_id] =  [ x.id for x in allocate_items_list] 
        #车辆未处理完订单字典{item_id:{item}}（可能需要卸载或装载订单）
        vehicles_unallocated_dict[vehicle_id] = { allocate_item.id : allocate_item for allocate_item in allocate_items_list }
        
    
    #车辆目的地和计划路线
    vehicle_id_to_destination = {vehicle_id:None  for vehicle_id,vehicle in id_to_vehicle.items()}
    vehicle_id_to_planned_route = {vehicle_id:[]  for vehicle_id,vehicle in id_to_vehicle.items()}
    #记录当前车辆距离和延迟，0是延迟，1是距离
    cur_vehicle_distance_delaytime ={vehicle_id:[0,0]  for vehicle_id,vehicle in id_to_vehicle.items()}
    #分派订单
    for item_id,item in id_to_unallocated_order_item.items():#未分配的订单
        if item_id in pre_matching_item_ids:#如果订单已经在[已分配订单，未到装货站(上一步)，分配过了，则跳过]
            continue
        #存储假设车辆信息
        set_vehicle_id_to_information={}
        for vehicle_id,vehicle in id_to_vehicle.items():
            import pdb
            #pdb.set_trace()
        #未装载订单copy，
            copy_vehicle_unallocated_dict={}
            copy_vehicle_unallocated_dict = copy.deepcopy(vehicles_unallocated_dict[vehicle_id])
            copy_vehicle_unallocated_dict[item_id] = item
            # 设定分配未装载订单 开始时间和当前位置（以destination离开时间或cur_factory离开时间为基准）
            if vehicle.destination is not None:
                allocate_time = vehicle.destination.leave_time
                cur_position = vehicle.destination.id 
            else:
                allocate_time = vehicle.leave_time_at_current_factory
                cur_position = vehicle.cur_factory_id
            
            
            #订单剩余分配次数（2代表还未装载，1代表已经装，0代表分配完毕）
            items_residue_times={}
             #从车辆copy_unallocate_item_id中录入,包含当前item
            items_residue_times ={unallocated_item_id:2 for unallocated_item_id,unallocated_item  in copy_vehicle_unallocated_dict.items()}
            #减去车辆已装货物，得到还需要分配次数
            for x in vehicles_loaded_items[vehicle_id]:
                items_residue_times[x] = 1
            
            #未分配订单id
            #all_items_id = np.array[x_id for x_id,x in items_residue_times.items()]

            
            #开始编码
            #固定已分配资源
            code = [ x for x in vehicles_loaded_items[vehicle_id]]
            # 可选分支集合
            branch_dict = {}
            for x in range(len(code)-1):
                branch_dict[x]=[]
            #有编码或没有编码的时候不同
            if len(code) !=0:
                branch_dict[len(code)-1]=[x_id for x_id,x in items_residue_times.items() if x >0]
           
            #累计延迟时间
            delay_time = 0
            sum_distance = 0
            min_delay_time =float('inf')
            min_distance = float('inf')
            good_route = []
            #给一个非零值
            sum_branch=1
    #-----------------------------------------------------应该测试本部分------------------------------------------------------------
            item_allocatetime_factory_delaytime_sumdistance_dict={}
        #存储过往 时间、地点、延迟、距离 dict
            if len(code) !=0:
                item_allocatetime_factory_delaytime_sumdistance_dict = {x:{} for x in range(len(code)-1) }
                item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = {'allocate_time':allocate_time,'cur_position':cur_position,
                                                                                     'delay_time':delay_time,'sum_distance':sum_distance}
            first_item_allocatetime_factory_delaytime_sumdistance_dict={'allocate_time':allocate_time,'cur_position':cur_position,
                                                                        'delay_time':delay_time,'sum_distance':sum_distance}
            succeed=False
            first = 0
            
            while sum_branch >= 0 and first <= len(copy_vehicle_unallocated_dict):#总分支数为0
                #pdb.set_trace()
                #当前无编码
                if len(code) == 0 and first < len(copy_vehicle_unallocated_dict):
                    #one 让sum_branch=0时
                    one = 0
                    cur_branch_item_id = all_items_id[first]
                    cur_branch_item = copy_vehicle_unallocated_dict[cur_branch_item_id]
                    copy_code = [cur_branch_item_id]
                    #检查环节
                    if check_sequence(all_items_id,copy_code) == True:
                        #检查车载
                        if check_capacity(items_residue_times,cur_branch_item_id,copy_vehicle_unallocated_dict,vehicle.board_capacity) == True:
                            #记录时间、延迟、位置、距离
                            allocate_time,delay_time,cur_position,sum_distance = record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,
                                                                                             cur_branch_item_id,cur_branch_item,route_map)
                            #更新编码、剩余订单次数、
                            code = copy_code
                            items_residue_times[cur_branch_item_id] -= 1
                            #记录当前编码的各项数据
                            item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = {'allocate_time':allocate_time,'cur_position':cur_position,
                                                                                                 'delay_time':delay_time,'sum_distance':sum_distance}
                            #增加下一分支可选集合信息
                            branch_dict[0] = [x_id for x_id,x in items_residue_times.items() if x >0]
                    first += 1
                
                #选择下一个分支
                elif len(branch_dict[len(branch_dict)-1]) != 0:
                    #当前item
                    cur_branch_item_id = branch_dict[len(branch_dict)-1][0]
                    cur_branch_item = copy_vehicle_unallocated_dict[cur_branch_item_id]
                    #删除当前分支
                    branch_dict[len(branch_dict)-1].remove(cur_branch_item_id) 
                    #编码副本,增加当前item
                    copy_code = copy.copy(code)
                    copy_code.append(cur_branch_item_id)               
                    #检查环节
                    if check_sequence(all_items_id,copy_code) == True:
                        #检查车载
                        if check_capacity(items_residue_times,cur_branch_item_id,copy_vehicle_unallocated_dict,vehicle.board_capacity) == True:
                            #记录时间、延迟、位置、距离
                            allocate_time,delay_time,cur_position,sum_distance = record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,
                                                                                             cur_branch_item_id,cur_branch_item,route_map)
                            #更新编码、剩余订单次数、
                            code = copy_code
                            items_residue_times[cur_branch_item_id] -= 1
                            #记录当前编码的各项数据
                            item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = {'allocate_time':allocate_time,'cur_position':cur_position,
                                                                                                 'delay_time':delay_time,'sum_distance':sum_distance}
                            #增加下一分支可选集合信息
                            branch_dict[len(code)-1] = [x_id for x_id,x in items_residue_times.items() if x >0]
                            
                else:#最后一个分支已经没有可选分支，。
                    print(code)
                    #记录延迟时间、距离、路线
                    #判读是否更新路线、距离、延迟时间,若是完整编码,包含所有item的装卸，则更新。
                    if min_delay_time >=  delay_time and min_distance > sum_distance and len(code)==len( items_residue_times)*2:
                        succeed=True
                        min_delay_time = delay_time
                        min_distance = sum_distance
                        good_route = copy.copy(code)
                    #最后一个分支已经没有可选单位，形成前一个分支的code、直到前一个分支有可选单位
                    for i in range(len(branch_dict)):
                        if len(branch_dict[len(branch_dict)-1]) == 0:
                            del branch_dict[len(code)-1]
                            del item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]
                            code.pop()
                        else:
                            break
    
                    #更新items_residue_times
                    items_residue_times ={unallocated_item_id:2 for unallocated_item_id,unallocated_item  in copy_vehicle_unallocated_dict.items()}
                    for x in code:
                        items_residue_times[x]-=1
                    #更新当前时间。当前地点,延迟和总距离
                    if len(code)!=0:
                        allocate_time=item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]['allocate_time']
                        cur_position = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]['cur_position']
                        delay_time = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]['delay_time']
                        sum_distance = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]['sum_distance']
                    else:
                        allocate_time=first_item_allocatetime_factory_delaytime_sumdistance_dict['allocate_time']
                        cur_position = first_item_allocatetime_factory_delaytime_sumdistance_dict['cur_position']
                        delay_time = first_item_allocatetime_factory_delaytime_sumdistance_dict['delay_time']
                        sum_distance = first_item_allocatetime_factory_delaytime_sumdistance_dict['sum_distance']
                    #------------------------
                    #pdb.set_trace()
              
                #剩余分支数
                sum_branch = 0
                for x_id,x in branch_dict.items():
                    sum_branch += len(x)
                if sum_branch ==0:
                    one +=1
                    if one == 2 and first == len(copy_vehicle_unallocated_dict):
                        sum_branch -=1
                    #----------------------
                #import pdb
                #pdb.set_trace()
                if first == len(copy_vehicle_unallocated_dict) and succeed==True:
                    #最优路线调整，删除已经装载物料。
                    if pro_code_type[vehicle_id] == 1:
                        del good_route[0:len([item for item in vehicle.carrying_items.items if item not in vehicle.destination.delivery_items ])]
                        for i in range(len(vehicle.destination.delivery_items)):
                            good_route.insert(0,vehicle.destination.delivery_items[len(vehicle.destination.delivery_items)-1-i].id)
                    
                    #无目的地，携带货物，只需删除携带货物
                    elif pro_code_type[vehicle_id] == 2:
                        del good_route[0:len(allocate_items_list)]
                    
                    #本车分支界定结束，存入最优解
                        #计划路线unallocated应加上 destination中的delivery_items
                    practical_vehicle_unallocated_dict=copy.deepcopy(copy_vehicle_unallocated_dict)
                    if vehicle.destination is not None:
                        for  item in vehicle.destination.delivery_items:
                            practical_vehicle_unallocated_dict[item.id] = item
                    origin_planned_route =  vehicle_planned_route(practical_vehicle_unallocated_dict,good_route,id_to_factory)
                    
                    # Combine adjacent-duplicated nodes.
                    origin_planned_route=__combine_duplicated_nodes(origin_planned_route)
                    
                    destination = None
                    planned_route = []
                    # determine the destination
                    if vehicle.destination is not None:
                        if len(origin_planned_route) == 0:
                            logger.error(f"Planned route of vehicle {vehicle_id} is wrong")
                        else:
                            destination = origin_planned_route[0]
                            destination.arrive_time = vehicle.destination.arrive_time#模型给的车辆到达目的地时间，赋值给到达时间
                            planned_route = [origin_planned_route[i] for i in range(1, len(origin_planned_route))]#目的地从计划路线排除
                    elif len(origin_planned_route) > 0:
                        destination = origin_planned_route[0]
                        planned_route = [origin_planned_route[i] for i in range(1, len(origin_planned_route))]
                    
                    set_vehicle_id_to_information[vehicle_id] = {'min_delay_time':min_delay_time,'min_distance':min_distance,
                                                                 'planned_route':planned_route,'destination':destination}
    
                    #vehicle_id_to_destination[vehicle_id] = destination
                
        #对比各车最优方案
        min_score = float('inf')
        for vehicle_id,vehicle in id_to_vehicle.items():
            cur_delaytime=cur_vehicle_distance_delaytime[vehicle_id][0]
            cur_distance=cur_vehicle_distance_delaytime[vehicle_id][1]
            score_1=set_vehicle_id_to_information[vehicle_id]['min_delay_time']-cur_delaytime
            score_2=set_vehicle_id_to_information[vehicle_id]['min_distance']-cur_distance
            score = score_1*Configs.LAMDA / 3600+score_2
            if score < min_score:
                min_score = score
                select_vehicle_id = vehicle_id
        #pdb.set_trace()
        vehicle_id_to_planned_route[select_vehicle_id]=set_vehicle_id_to_information[select_vehicle_id]['planned_route']
        vehicle_id_to_destination[select_vehicle_id]=set_vehicle_id_to_information[select_vehicle_id]['destination']
        cur_vehicle_distance_delaytime[vehicle_id][0]=set_vehicle_id_to_information[select_vehicle_id]['min_delay_time']
        cur_vehicle_distance_delaytime[vehicle_id][1]=set_vehicle_id_to_information[select_vehicle_id]['min_distance']
        vehicles_unallocated_dict[select_vehicle_id] = copy_vehicle_unallocated_dict
        
        #其他车目的地
    
    for vehicle_id,vehicle in id_to_vehicle.items():
        if  vehicle_id_to_destination[vehicle_id] == None and vehicle.destination is not None:
            vehicle_id_to_destination[vehicle_id]=[vehicle.destination]
    

    return vehicle_id_to_destination,vehicle_id_to_planned_route

#判定该分支是否继续
def __judge_score( min_delay_time,min_distance,delay_time,sum_distance):
    judge=True
    cur_score = min_delay_time*Configs.LAMDA/3600 + min_distance
    new_score = delay_time*Configs.LAMDA/3600 + sum_distance
    if new_score > cur_score:
        judge = False
    return judge

def dispatch_orders_to_vehicle(id_to_factory, id_to_unallocated_order_item, id_to_ongoing_order_item, id_to_vehicle,route_map):
    #创建车辆 未分配的item_id信息字典（包括所有已携带订单，因为未装载的可以重新分配顺序） eg ；{ '2300460049-1':<src.common.order.OrderItem object at 0x00000173639C2520>}
    vehicles_unallocated_dict = { vehicle_id : {} for vehicle_id,vehicle in id_to_vehicle.items() }
    #车辆已经carrying货物（包括实际已装和假设在目的地装）
    vehicles_loaded_items={}
    pre_matching_item_ids = []
    pro_code_type = np.array([0 for i in range(len(id_to_vehicle))])
    i=0
    for vehicle_id,vehicle in id_to_vehicle.items():
        #路径顺序编码(根据item_id,order_id会重复)
            #分2种情况，case1：有目的地；case3:无目的地
        if vehicle.destination is not None:
            #应分配route的items（需卸载和装载）,当前携带items 除去即将卸载items,装上准备装的item
            pro_code_type[i] = 1
            allocate_items_list = [item for item in vehicle.carrying_items.items if item not in vehicle.destination.delivery_items ]
            pickup_items = vehicle.destination.pickup_items #取货单
            pre_matching_item_ids.extend([item.id for item in pickup_items])#在目的地准备装货订单
            allocate_items_list.extend([item for item in pickup_items])
            
            #case2:将携带item创建（有无皆可）
        elif  vehicle.destination is  None:
            pro_code_type[i] =2
            allocate_items_list = [item for item in vehicle.carrying_items.items  ]
        
        #车辆假设已经装载 itme 的id（实际已装载和即将在目的地装载）
        vehicles_loaded_items[vehicle_id] =  [ x.id for x in allocate_items_list] 
        #车辆未处理完订单字典{item_id:{item}}（可能需要卸载或装载订单）
        vehicles_unallocated_dict[vehicle_id] = { allocate_item.id : allocate_item for allocate_item in allocate_items_list }
        i+=1

    #车辆目的地和计划路线
    vehicle_id_to_destination = {vehicle_id:None  for vehicle_id,vehicle in id_to_vehicle.items()}
    vehicle_id_to_planned_route = {vehicle_id:[]  for vehicle_id,vehicle in id_to_vehicle.items()}
    #记录当前车辆距离和延迟，0是延迟，1是距离
    cur_vehicle_distance_delaytime = {}
    for vehicle_id,vehicle in id_to_vehicle.items():
        if vehicle.destination is not  None:
            x_factory_id = vehicle.destination.id
            x_time = vehicle.destination.leave_time
        else:
            x_factory_id = vehicle.cur_factory_id
            x_time = vehicle.leave_time_at_current_factory
        cur_vehicle_distance_delaytime[vehicle_id] =[0,0,x_time,x_factory_id]
    #建立未分配订单集合
    id_to_unallocated_order_not_in_pre = {}
    for item_id,item in id_to_unallocated_order_item.items():
        if item_id not in pre_matching_item_ids:
                id_to_unallocated_order_not_in_pre[item_id] = item
    #打包订单号相同的item
    pack_id_to_unallocated_order_item , pack_id_to_same_order = __combine_same_order( id_to_unallocated_order_not_in_pre)
    #复制未装载车辆信息
    copy_vehicles_unallocated_dict={}

    #如果超过8个将后面的取出，按时间规则加在后面
    save_item_judge = False
    if len(pack_id_to_same_order) > 6:
        save_item_judge = True
        save_item_num = len(pack_id_to_same_order)-6
        save_pack_id_to_unallocated_order_item={}
        save_pack_id_to_same_oreder={}
        i=0
        save_item_list = []
        for x_id,x in pack_id_to_same_order.items():
            save_item_list.append(x_id)
        for i in range(save_item_num):
            item_id =save_item_list.pop()    
            save_pack_id_to_same_oreder[item_id] = pack_id_to_same_order[item_id]
            save_pack_id_to_unallocated_order_item[item_id] = pack_id_to_unallocated_order_item[item_id]
            del pack_id_to_same_order[item_id]
            del pack_id_to_unallocated_order_item[item_id]

    #分派订单----------------------------------------------------------------------------------------------------------------------------
    for item_id,item in pack_id_to_unallocated_order_item.items():#未分配的订单
                                                                                

        #存储假设车辆信息
        set_vehicle_id_to_information={}
        vehicle_index = 0
        for vehicle_id,vehicle in id_to_vehicle.items():
            import pdb
            #pdb.set_trace()
        #未装载订单copy，
            copy_vehicle_unallocated_dict={}
            copy_vehicle_unallocated_dict = copy.deepcopy(vehicles_unallocated_dict[vehicle_id])
            copy_vehicle_unallocated_dict[item_id] = item
            copy_vehicles_unallocated_dict[vehicle_id]= copy_vehicle_unallocated_dict
            # 设定分配未装载订单 开始时间和当前位置（以destination离开时间或cur_factory离开时间为基准）
            if vehicle.destination is not None:
                allocate_time = vehicle.destination.leave_time
                cur_position = vehicle.destination.id 
            else:
                allocate_time = vehicle.leave_time_at_current_factory
                cur_position = vehicle.cur_factory_id
            
            
            #订单剩余分配次数（2代表还未装载，1代表已经装，0代表分配完毕）
            items_residue_times={}
            #从车辆copy_unallocate_item_id中录入,包含当前item
            items_residue_times ={unallocated_item_id:2 for unallocated_item_id,unallocated_item  in copy_vehicle_unallocated_dict.items()}
            #未分配订单id 
            all_items_id = [x_id for x_id,x in items_residue_times.items()]
            #减去车辆已装货物，得到还需要分配次数
            for x in vehicles_loaded_items[vehicle_id]:
                items_residue_times[x] = 1
            
            #开始编码
            #固定已分配资源
            code = [ x for x in vehicles_loaded_items[vehicle_id]]
            # 可选分支集合
            branch_dict = {}
            for x in range(len(code)-1):
                branch_dict[x]=[]
            #有编码或没有编码的时候不同
            if len(code) !=0:
                first_code_fixed = True
                branch_dict[len(code)-1]=[x_id for x_id,x in items_residue_times.items() if x >0]
            else:
                first_code_fixed = False
            #累计延迟时间
            delay_time = 0
            sum_distance = 0
            min_delay_time =float('inf')
            min_distance = float('inf')
            good_route = []
            #给一个非零值
            sum_branch=1
    #-----------------------------------------------------该-----------------------------------------------------------
            item_allocatetime_factory_delaytime_sumdistance_dict = [[] for i in range(len( items_residue_times)*2)]
            
        #存储过往 allocate_time时间、cur_position地点、delay_time延迟、sum_distance距离 dict
            if len(code) !=0:   
                item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = [allocate_time,cur_position,
                                                                                    delay_time,sum_distance]
            first_item_allocatetime_factory_delaytime_sumdistance_dict=[allocate_time,cur_position,delay_time,sum_distance]
            succeed=False
            first = 0
            one=0
            while sum_branch >= 0 and first <= len(copy_vehicle_unallocated_dict):#总分支数为0
                if first_code_fixed == True and one != 1:
                    one = 0
                #当前无编码
                if len(code) == 0 and first < len(copy_vehicle_unallocated_dict) and first_code_fixed == False:
                    #one 让sum_branch=0时
                    one = 0
                    cur_branch_item_id = all_items_id[first]
                    cur_branch_item = copy_vehicle_unallocated_dict[cur_branch_item_id]
                    copy_code = [cur_branch_item_id]
                    #检查环节
                    if check_sequence(all_items_id,copy_code) == True:
                        #检查车载
                        if check_capacity(items_residue_times,cur_branch_item_id,copy_vehicle_unallocated_dict,vehicle.board_capacity) == True:
                            #记录时间、延迟、位置、距离
                            allocate_time,delay_time,cur_position,sum_distance = record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,
                                                                                            cur_branch_item_id,cur_branch_item,route_map)
                            #判定该分支是否需要继续
                            if __judge_score(min_delay_time,min_distance,delay_time,sum_distance) == True:
                                #更新编码、剩余订单次数、
                                code = copy_code
                                items_residue_times[cur_branch_item_id] -= 1
                                #记录当前编码的各项数据
                                item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = [allocate_time,cur_position,delay_time,sum_distance]
                                #增加下一分支可选集合信息
                                branch_dict[0] = [x_id for x_id,x in items_residue_times.items() if x >0]
                                

                    first += 1
                
                #选择下一个分支
                elif len(code) !=0 and len(branch_dict[len(branch_dict)-1]) != 0 :
                    #当前item
                    cur_branch_item_id = branch_dict[len(branch_dict)-1][0]
                    cur_branch_item = copy_vehicle_unallocated_dict[cur_branch_item_id]
                    #删除当前分支
                    branch_dict[len(branch_dict)-1].remove(cur_branch_item_id) 
                    #编码副本,增加当前item
                    copy_code = copy.copy(code)
                    copy_code.append(cur_branch_item_id)               
                    #检查环节
                    if check_sequence(all_items_id,copy_code) == True:
                        #检查车载
                        if check_capacity(items_residue_times,cur_branch_item_id,copy_vehicle_unallocated_dict,vehicle.board_capacity) == True:
                            #记录时间、延迟、位置、距离
                            allocate_time,delay_time,cur_position,sum_distance = record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,
                                                                                            cur_branch_item_id,cur_branch_item,route_map)
                            #判定该分支是否需要继续
                            if __judge_score(min_delay_time,min_distance,delay_time,sum_distance) == True:
                                #更新编码、剩余订单次数、
                                code = copy_code
                                del copy_code
                                items_residue_times[cur_branch_item_id] -= 1
                                #记录当前编码的各项数据
                                item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1] = [allocate_time,cur_position,delay_time,sum_distance]
                                #增加下一分支可选集合信息
                                branch_dict[len(code)-1] = [x_id for x_id,x in items_residue_times.items() if x >0]
                            
                else:#最后一个分支已经没有可选分支。
                    #记录延迟时间、距离、路线
                    #判读是否更新路线、距离、延迟时间,若是完整编码,包含所有item的装卸，则更新。
                    if min_delay_time >=  delay_time and min_distance > sum_distance and len(code)==len( items_residue_times)*2:
                        succeed=True
                        min_allocate_time = allocate_time
                        min_cur_position = cur_position
                        min_delay_time = delay_time
                        min_distance = sum_distance
                        good_route = copy.copy(code)
                    #最后一个分支已经没有可选单位，形成前一个分支的code、直到前一个分支有可选单位
                    for i in range(len(branch_dict)):
                        if len(branch_dict[len(branch_dict)-1]) == 0:
                            del branch_dict[len(code)-1]
                            item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1]=[]
                            code.pop()
                        else:
                            break

                    #更新items_residue_times
                    items_residue_times ={unallocated_item_id:2 for unallocated_item_id,unallocated_item  in copy_vehicle_unallocated_dict.items()}
                    for x in code:
                        items_residue_times[x]-=1
                    #更新当前时间。当前地点,延迟和总距离
                    if len(code)!=0:
                        allocate_time=item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1][0]
                        cur_position = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1][1]
                        delay_time = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1][2]
                        sum_distance = item_allocatetime_factory_delaytime_sumdistance_dict[len(code)-1][3]
                    else:
                        allocate_time=first_item_allocatetime_factory_delaytime_sumdistance_dict[0]
                        cur_position = first_item_allocatetime_factory_delaytime_sumdistance_dict[1]
                        delay_time = first_item_allocatetime_factory_delaytime_sumdistance_dict[2]
                        sum_distance = first_item_allocatetime_factory_delaytime_sumdistance_dict[3]
                    #------------------------
                    #pdb.set_trace()
            
                #剩余分支数
                sum_branch = 0
                for x_id,x in branch_dict.items():
                    sum_branch += len(x)
                if sum_branch ==0:
                    one +=1
                    if first_code_fixed == True:
                        first = len(copy_vehicle_unallocated_dict)
                    if one == 2 and first == len(copy_vehicle_unallocated_dict):
                        sum_branch -=1

                        #如果初始化code存在固定编码，则不需要遍历所有unallocated。

                    #----------------------
                #import pdb
                #pdb.set_trace()
                if first == len(copy_vehicle_unallocated_dict) and succeed==True and sum_branch < 0:
                    #最优路线调整，删除已经装载物料。
                    if pro_code_type[vehicle_index] == 1:
                        del good_route[0:len([item for item in vehicle.carrying_items.items if item not in vehicle.destination.delivery_items ])]
                        for i in range(len(vehicle.destination.delivery_items)):
                            good_route.insert(0,vehicle.destination.delivery_items[len(vehicle.destination.delivery_items)-1-i].id)
                    
                    #无目的地，携带货物，只需删除携带货物
                    elif pro_code_type[vehicle_index] == 2:
                        del good_route[0:len(vehicle.carrying_items.items)]
                                                                                
                    #本车分支界定结束，存入最优解
                        #计划路线unallocated应加上 destination中的delivery_items
                    practical_vehicle_unallocated_dict = copy.copy(copy_vehicle_unallocated_dict)
                    if vehicle.destination is not None:
                        for  item in vehicle.destination.delivery_items:
                            practical_vehicle_unallocated_dict[item.id] = item            
                    for x_id,x in id_to_unallocated_order_item.items():
                        practical_vehicle_unallocated_dict[x_id]=x
                            
                    #将good route中打包item替换为全部item  
                    good_route = __replace_item(good_route,pack_id_to_same_order,pre_matching_item_ids)                                    
                    origin_planned_route =  vehicle_planned_route(practical_vehicle_unallocated_dict,good_route,id_to_factory)
                    
                    # Combine adjacent-duplicated nodes.
                    origin_planned_route=__combine_duplicated_nodes(origin_planned_route)
                    

                    destination = None
                    planned_route = []
                    # determine the destination
                    if vehicle.destination is not None:
                        if len(origin_planned_route) == 0:
                            logger.error(f"Planned route of vehicle {vehicle_id} is wrong")
                        else:
                            destination = origin_planned_route[0]
                            destination.arrive_time = vehicle.destination.arrive_time#模型给的车辆到达目的地时间，赋值给到达时间
                            planned_route = [origin_planned_route[i] for i in range(1, len(origin_planned_route))]#目的地从计划路线排除
                    elif len(origin_planned_route) > 0:
                        destination = origin_planned_route[0]
                        planned_route = [origin_planned_route[i] for i in range(1, len(origin_planned_route))]
                    
                    set_vehicle_id_to_information[vehicle_id] = {'min_delay_time':min_delay_time,'min_distance':min_distance,'planned_route':planned_route,
                                                                'destination':destination,'min_allocate_time':min_allocate_time,'min_cur_position':min_cur_position}
                    #清除内存


                    #vehicle_id_to_destination[vehicle_id] = destination
            vehicle_index += 1   
        #对比各车最优方案
        min_score = float('inf')
        for vehicle_id,vehicle in id_to_vehicle.items():
            cur_delaytime=cur_vehicle_distance_delaytime[vehicle_id][0]
            cur_distance=cur_vehicle_distance_delaytime[vehicle_id][1]
            score_1=set_vehicle_id_to_information[vehicle_id]['min_delay_time']-cur_delaytime
            score_2=set_vehicle_id_to_information[vehicle_id]['min_distance']-cur_distance
            score = score_1*Configs.LAMDA / 3600+score_2
            if score < min_score:
                min_score = score
                select_vehicle_id = vehicle_id
        #pdb.set_trace()
        vehicle_id_to_planned_route[select_vehicle_id]=set_vehicle_id_to_information[select_vehicle_id]['planned_route']
        vehicle_id_to_destination[select_vehicle_id]=set_vehicle_id_to_information[select_vehicle_id]['destination']
        cur_vehicle_distance_delaytime[vehicle_id][0]=set_vehicle_id_to_information[select_vehicle_id]['min_delay_time']
        cur_vehicle_distance_delaytime[vehicle_id][1]=set_vehicle_id_to_information[select_vehicle_id]['min_distance']
        cur_vehicle_distance_delaytime[vehicle_id][2]=set_vehicle_id_to_information[select_vehicle_id]['min_allocate_time']
        cur_vehicle_distance_delaytime[vehicle_id][3]=set_vehicle_id_to_information[select_vehicle_id]['min_cur_position']
        vehicles_unallocated_dict[select_vehicle_id] = copy_vehicles_unallocated_dict[select_vehicle_id]

    #把多余的取出来补到时间最短距离最小的地方
    if save_item_judge == True:
        for x_id,x in save_pack_id_to_same_oreder.items():
            append_route = [x_id,x_id]
            append_route=__replace_item(append_route,save_pack_id_to_same_oreder,pre_matching_item_ids)
            save_planned_route =  vehicle_planned_route(id_to_unallocated_order_item ,append_route,id_to_factory)  
            save_planned_route = __combine_duplicated_nodes(save_planned_route)
            cur_item_id = x_id
            cur_item = save_pack_id_to_unallocated_order_item[x_id]
            min_score = float('inf')
            for vehicle_id,vehicle in id_to_vehicle.items():
                sum_distance
                items_residue_times={x_id:2}
                delay_time = cur_vehicle_distance_delaytime[vehicle_id][0]
                sum_distance = cur_vehicle_distance_delaytime[vehicle_id][1]
                allocate_time = cur_vehicle_distance_delaytime[vehicle_id][2]
                cur_position = cur_vehicle_distance_delaytime[vehicle_id][3]
                cur_delaytime = delay_time
                cur_distance = sum_distance
                for i in range(2):
                    allocate_time,delay_time,cur_position,sum_distance = record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,cur_item_id,cur_item,route_map)
                    score_1=delay_time-cur_delaytime
                    score_2=sum_distance-cur_distance
                    score = score_1*Configs.LAMDA / 3600+score_2
                if score < min_score:
                    min_score = score
                    select_vehicle_id = vehicle_id
                    select_vehicle = vehicle
            if  select_vehicle.destination is not None:
                vehicle_id_to_destination[select_vehicle_id]=vehicle.destination
                vehicle_id_to_planned_route[select_vehicle_id].extend(save_planned_route)       
            elif vehicle_id_to_destination[select_vehicle_id] is not None:
                vehicle_id_to_planned_route[select_vehicle_id].extend(save_planned_route)
            elif select_vehicle.destination is None and vehicle_id_to_destination[select_vehicle_id] is None:
                vehicle_id_to_destination[select_vehicle_id] = save_planned_route[0]
                vehicle_id_to_planned_route[select_vehicle_id].append(save_planned_route[1])
            cur_vehicle_distance_delaytime[vehicle_id][0] = delay_time
            cur_vehicle_distance_delaytime[vehicle_id][1] = sum_distance
            cur_vehicle_distance_delaytime[vehicle_id][2] = allocate_time  
            cur_vehicle_distance_delaytime[vehicle_id][3] = cur_position  

    #其他车目的地和route
    for vehicle_id,vehicle in id_to_vehicle.items():
        if  vehicle_id_to_destination[vehicle_id] == None and vehicle.destination is not None:
            vehicle_id_to_destination[vehicle_id]=vehicle.destination
        #如果存在未处理订单，且未加入新订单，补上计划路线
        if len(vehicle_id_to_planned_route[vehicle_id]) == 0 and len(vehicles_unallocated_dict[vehicle_id]) != 0:
            ensure_route = [0 for i in range(len(vehicles_unallocated_dict[vehicle_id]))]
            i=0
            for x in vehicles_unallocated_dict[vehicle_id].keys():
                ensure_route[len(ensure_route)-1-i]=x
                i+=1
            origin_planned_route =  vehicle_planned_route(vehicles_unallocated_dict[vehicle_id],ensure_route,id_to_factory)    
                # Combine adjacent-duplicated nodes.
            origin_planned_route=__combine_duplicated_nodes(origin_planned_route)
            vehicle_id_to_planned_route[vehicle_id]=origin_planned_route

    
    return vehicle_id_to_destination,vehicle_id_to_planned_route

"""

Main body
# Note
# This is the demo to show the main flowchart of the algorithm

"""

def scheduling():
    # read the input json, you can design your own classes
    id_to_factory, id_to_unallocated_order_item, id_to_ongoing_order_item, id_to_vehicle,route_map = __read_input_json()

    # dispatching algorithm
    vehicle_id_to_destination, vehicle_id_to_planned_route = dispatch_orders_to_vehicle(id_to_factory, id_to_unallocated_order_item, 
                                                                                        id_to_ongoing_order_item, id_to_vehicle,route_map)

    # output the dispatch result
    __output_json(vehicle_id_to_destination, vehicle_id_to_planned_route)

#读工厂信息
def __read_input_json():
    # read the factory info
    id_to_factory = get_factory_info(Configs.factory_info_file_path)

    #read the route map
    code_to_route = get_route_map(Configs.route_info_file_path)
    route_map = Map(code_to_route)

    # read the input json, you can design your own classes
    unallocated_order_items = read_json_from_file(Configs.algorithm_unallocated_order_items_input_path)
    id_to_unallocated_order_item = get_order_item_dict(unallocated_order_items, 'OrderItem')

    ongoing_order_items = read_json_from_file(Configs.algorithm_ongoing_order_items_input_path)
    id_to_ongoing_order_item = get_order_item_dict(ongoing_order_items, 'OrderItem')

    id_to_order_item = {**id_to_unallocated_order_item, **id_to_ongoing_order_item}

    vehicle_infos = read_json_from_file(Configs.algorithm_vehicle_input_info_path)
    id_to_vehicle = get_vehicle_instance_dict(vehicle_infos, id_to_order_item, id_to_factory)

    return id_to_factory, id_to_unallocated_order_item, id_to_ongoing_order_item, id_to_vehicle,route_map

#输出文件
def __output_json(vehicle_id_to_destination, vehicle_id_to_planned_route):
    write_json_to_file(Configs.algorithm_output_destination_path, convert_nodes_to_json(vehicle_id_to_destination))
    write_json_to_file(Configs.algorithm_output_planned_route_path, convert_nodes_to_json(vehicle_id_to_planned_route))


"""          
try:   
    !jupyter nbconvert --to python new_algorithm.ipynb
    # python即转化为.py，script即转化为.html
    # file_name.ipynb即当前module的文件名
except:
    pass        
"""