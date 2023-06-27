import gtpyhop
import time 
import random
from state.rigid import rigid

def transfer(state, agent, loc_from, loc_to, traj_client):
    if agent in state.agents and loc_from in state.locations and loc_to in state.locations and state.at[agent] == loc_from:
        if agent == 'robot':
            if len(rigid.locations[loc_from])>1:
                if not traj_client.transfer([rigid.locations[loc_from][0]]):
                    return None
            if not traj_client.transfer(rigid.locations[loc_to]):
                return None
        state.at[agent] = loc_to
        if state.holding[agent]:
            obj = state.holding[agent]
            state.at[obj] = loc_to
        return state

def grasp(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.at[agent] == state.at[obj] and not state.holding[agent]:
        if agent == 'robot':
            traj_client.traj_p.close_gripper()
        if agent == 'human' and obj == 'box':
            state.holding[agent] = None#'screws'
        else:
            state.holding[agent] = obj
        return state

def wait_empty_box(state, traj_client):
    if state.box_empty == False:
        traj_client.camera_activation_pub.publish(True)
        if traj_client.stop_sleeping_sig.is_set():
            traj_client.stop_sleeping_sig.clear()
        traj_client.stop_sleeping_sig.wait()
        state.box_empty = True
        return state

def release(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.holding[agent] == obj:
        if agent == 'robot':
            if state.at[agent] == 'exchange point':
                traj_client.melexis_activation_pub.publish(True)
                if traj_client.stop_sleeping_sig.is_set():
                    traj_client.stop_sleeping_sig.clear()
                traj_client.stop_sleeping_sig.wait()

        traj_client.traj_p.open_gripper()
        state.holding[agent] = None
        return state 

def check_available_obj(state, obj_list, traj_client):
    if set(obj_list) <= state.objects:
        # use camera to check what objects are available
        traj_client.aruco_activation_pub.publish(True)
        if traj_client.stop_sleeping_sig.is_set():
            traj_client.stop_sleeping_sig.clear()
        traj_client.stop_sleeping_sig.wait()
        if len(traj_client.current_obj) == 1 and traj_client.current_obj[0] == '':
            traj_client.current_obj = []
        
        # dict =  {obj1: pos1, obj2: pos2}

        print('available objects: ', traj_client.current_obj)
        state.available_objects = traj_client.current_obj
        # state.available_objects = dict.keys()
        # for k in dict.keys():
        #     state.at[k] = dict[k]
        return state

def choose_obj(state):
    if state.available_objects:
        length = len(state.available_objects)
        if length == 0:
            state.selected_object = None
        elif length == 1:
            state.selected_object = state.available_objects[0]
            # state.available_objects = []
        else:
            state.selected_object = random.choice(state.available_objects)
            # state.available_objects.remove(state.selected_object)
        return state

gtpyhop.declare_actions(transfer, grasp, release, wait_empty_box, check_available_obj, choose_obj)