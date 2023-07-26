import gtpyhop
import time 
import random
import rospy
from state.rigid import rigid

def transfer(state, agent, loc_to, traj_client):
    if agent in state.agents and loc_to in state.locations:
        loc_from = state.at[agent] 
        loc_to_pose = rigid.locations[loc_to]

        if agent == 'robot':
            if loc_to == 'exchange point' and state.active_arm[agent] == 'left':
                for i in range(len(loc_to_pose)):
                    loc_to_pose[i].position.y *= -1.0
            if len(rigid.locations[loc_from])>1:
                if not traj_client.transfer([rigid.locations[loc_from][0]], state.active_arm[agent]):
                    return None
            if not traj_client.transfer(loc_to_pose, state.active_arm[agent]):
                return None
            
            # here we should call the wrist camera
            # get the precise position of the object
            # call again transfer
            # we will have something like --- brick1_pose': [pose1] from the previous methods
            # so we have to add the precise pose in the second position of the list
            # brick1_pose': [pose1, precise],
            if state.selected_object:
                if 'brick' in state.selected_object and loc_to != 'workspace':
                    print('selected_object: ', state.selected_object)
                    selected_marker_id = int(state.obj2markerID[state.selected_object])
                    precise_pose = precision_marker_detection(state, traj_client, selected_marker_id, state.active_arm[agent])
                    print(precise_pose)
                    rigid.locations[state.selected_object + '_pose'].append(precise_pose)
                    print('new location for ', state.selected_object, ' is: ', rigid.locations[state.selected_object + '_pose'])
                    if not traj_client.transfer([precise_pose], state.active_arm[agent]):
                        return None

        state.at[agent] = loc_to
        state.at[state.active_arm[agent]] = loc_to
        if state.holding[agent]:
            obj = state.holding[agent]
            state.at[obj] = loc_to
        return state
    

def tuck_arms(state, agent, traj_client):
    if agent in state.agents and not state.holding[agent]:
        state.at[agent] = 'tuck_positionR'
        state.at['left'] = 'tuck_positionL'
        state.at['right'] = 'tuck_positionR'

        if not traj_client.transfer([rigid.locations['tuck_positionR'][0]], 'right'):
            return None 
        if not traj_client.transfer([rigid.locations['tuck_positionL'][0]], 'left'):
            return None
    return state


def grasp(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.at[agent] == state.at[obj]: #and not state.holding[agent]:
        if agent == 'robot':
            if not state.holding[agent]:
                traj_client.traj_p.close_gripper(state.active_arm[agent])
                state.holding[agent] = obj
        if agent == 'human':
            if obj == 'box':
                state.holding[agent] = None#'screws'
            else:      
                state.holding[agent] = obj
        return state
    
def precision_marker_detection(state, traj_client, id, side):
    print('precision_marker_detection')
    traj_client.baxter_camera_activation_pub.publish(f'{id}_{side}')
    # traj_client.baxter_camera_activation_pub.publish(id)
    print('activating baxter camera with id: ', id)
    if traj_client.stop_sleeping_sig.is_set():
        traj_client.stop_sleeping_sig.clear()
    traj_client.stop_sleeping_sig.wait()
    return traj_client.precise_m_p

def wait_empty_box(state, traj_client):
    if state.box_empty == False:
        # TODO UNCOMMENT AND TEST
        traj_client.camera_activation_pub.publish(state.active_arm['robot'])
        # traj_client.camera_activation_pub.publish(True)
        if traj_client.stop_sleeping_sig.is_set():
            traj_client.stop_sleeping_sig.clear()
        traj_client.stop_sleeping_sig.wait()
        state.box_empty = True
        return state
    
def wait_tool_pulling(state, agent, traj_client):
    if agent in state.agents:
        if agent == 'robot':
            if state.active_arm[agent] == 'right':
                traj_client.melexis_activation_pub.publish(True)
                if traj_client.stop_sleeping_sig.is_set():
                    traj_client.stop_sleeping_sig.clear()
                traj_client.stop_sleeping_sig.wait()
                traj_client.traj_p.open_gripper('right')
            elif state.active_arm[agent] == 'left':
                traj_client.traj_p.open_gripper('left')
    return state

def wait_idle(state, traj_client):
    traj_client.idle_classification_pub.publish(True)
    if traj_client.stop_sleeping_sig.is_set():
        traj_client.stop_sleeping_sig.clear()
    traj_client.stop_sleeping_sig.wait()
    print('not idle anymore')
    return state

def release(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and state.holding[agent] == obj:
        if agent == 'robot':
            # if state.holding[agent] != 'box':
            #     if state.active_arm[agent] == 'right':
            #         traj_client.melexis_activation_pub.publish(True)
            #         if traj_client.stop_sleeping_sig.is_set():
            #             traj_client.stop_sleeping_sig.clear()
            #         traj_client.stop_sleeping_sig.wait()
            #         traj_client.traj_p.open_gripper('right')

            #     # TO TEST
            #     if state.active_arm[agent] == 'left':
            #         traj_client.traj_p.open_gripper(state.active_arm[agent])

            # elif state.holding[agent] == 'box':
            #     traj_client.traj_p.open_gripper(state.active_arm[agent])
            traj_client.traj_p.open_gripper(state.active_arm[agent])
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
            traj_client.active_marker_poses = {}
        
        state.available_objects = []
        # dict =  {0: pos1, 10: pos2, 100: pos3}
        # listofnames = ['brick1', 'brick2', 'brick3']
        print('available markers: ', traj_client.current_obj)
        print('active_marker_poses: ', traj_client.active_marker_poses)
        print('state.available_objects: ', state.available_objects)
        for obj_marker_id in traj_client.current_obj:
            if state.markerID2obj[obj_marker_id] not in state.available_objects:
                state.available_objects.append(state.markerID2obj[obj_marker_id])
            rigid.locations[state.markerID2obj[obj_marker_id] + '_pose'] = [traj_client.active_marker_poses[obj_marker_id]]
            state.at[state.markerID2obj[obj_marker_id]] = state.markerID2obj[obj_marker_id] + '_pose'

        print('available objects: ', state.available_objects)
        print('obj2pose: ', rigid.locations)
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


def choose_arm(state, obj, agent):
    if agent in state.agents and obj in state.objects:
        if agent == 'human':
            return state
        elif agent == 'robot':
            loc = state.at[obj]
            if rigid.locations[loc][0].position.y > 0:
                state.active_arm[agent] = 'left'
                # rospy.logwarn('Choosing left arm')
                # rospy.logwarn(loc)
                # rospy.logwarn(rigid.locations[loc][0].position)
            else:
                state.active_arm[agent] = 'right'
                # rospy.logwarn('Choosing right arm')
                # rospy.logwarn(loc)
                # rospy.logwarn(rigid.locations[loc][0].position)

            state.at[agent] = state.at[state.active_arm[agent]]
        return state
    
def reset_active_arm(state, agent):
    if agent in state.agents:
        if agent == 'human':
            return state
        elif agent == 'robot':
            state.active_arm[agent] = None
        return state
    
def reset_selected_object(state):
    state.selected_object = None
    return state

gtpyhop.declare_actions(transfer, 
                        grasp, 
                        release, 
                        wait_tool_pulling, 
                        wait_empty_box,
                        wait_idle, 
                        check_available_obj,
                        choose_obj, 
                        choose_arm, 
                        reset_active_arm, 
                        reset_selected_object,
                        tuck_arms)