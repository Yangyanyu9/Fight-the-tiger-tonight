#!/usr/bin/env python
# coding: utf-8

# In[11]:


import copy

class Stack(object):

    def __init__(self):
        self.stack = []

    def push(self, data):
        """
        进栈函数
        """
        self.stack.append(data)

    def pop(self):
        """
        出栈函数，
        """
        return self.stack.pop()

    def gettop(self):
        """
        取栈顶
        """
        return self.stack[-1]
    
def check_sequence(string,code):
    check_sequence_result=True
    stack = Stack()
    #code = ['2300460049-1','2300460049-2','2300460049-3','2300460049-4','2300460049-5','2300460049-6','2300460049-1']
    #string = ['2300460049-1','2300460049-2','2300460049-3','2300460049-4','2300460049-5','2300460049-6']
    lift_list = copy.copy(string)
    right_list = copy.copy(string)
    for item_id in code:
        if item_id in lift_list:
            stack.push(item_id)
            lift_list.remove(item_id)
        elif item_id in right_list and len(stack.stack)>0 :
            if stack.gettop() == item_id:
                stack.pop()
            else:
                check_sequence_result = False
    return check_sequence_result

def check_capacity(items_residue_times:dict,cur_branch_item:str,copy_vehicle_unallocated_dict:dict,vehicle_capacity:int):
    check_capacity_result=True
    
    if items_residue_times[cur_branch_item] == 2:
        loaded_items_capacity = 0
        for x_id,x_times in items_residue_times.items():
            if x_times == 1:
                loaded_items_capacity +=copy_vehicle_unallocated_dict[x_id].demand
        loaded_items_capacity += copy_vehicle_unallocated_dict[cur_branch_item].demand
        if vehicle_capacity < loaded_items_capacity:
            check_capacity_result = False
    return check_capacity_result

def record_time(allocate_time,cur_position,delay_time,sum_distance,items_residue_times,cur_branch_item_id,cur_branch_item,route_map):
    if items_residue_times[cur_branch_item_id] == 2:
        #当前位置到pick_up_factory,运输时间
        transport_time = route_map.calculate_transport_time_between_factories(cur_position,cur_branch_item.pickup_factory_id)
        #运输距离
        transport_distance = route_map.calculate_distance_between_factories(cur_position,cur_branch_item.pickup_factory_id)
        sum_distance +=transport_distance
        #到达新目的地时间
        allocate_time += transport_time
        #装载时间
        allocate_time += cur_branch_item.load_time
        #当前所在位置
        cur_position = cur_branch_item.pickup_factory_id
    elif items_residue_times[cur_branch_item_id] == 1:

        #当前位置到delivery_factory，运输时间
        transport_time = route_map.calculate_transport_time_between_factories(cur_position,cur_branch_item.delivery_factory_id)
        #运输距离
        transport_distance = route_map.calculate_distance_between_factories(cur_position,cur_branch_item.delivery_factory_id)
        sum_distance +=transport_distance
        #到达新目的地时间
        allocate_time += transport_time
        #卸载后时间
        allocate_time += cur_branch_item.unload_time
        #累计延迟时间
        delay_time += max(0,allocate_time - cur_branch_item.committed_completion_time)
        #当前所在位置
        cur_position = cur_branch_item.delivery_factory_id  
          
    return allocate_time,delay_time,cur_position,sum_distance