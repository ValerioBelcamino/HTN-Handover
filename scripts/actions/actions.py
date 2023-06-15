import gtpyhop
import time 
from state.rigid import rigid
def transfer(state, agent, loc_from, loc_to, traj_client):
    if agent in state.agents and loc_from in state.locations and loc_to in state.locations and state.at[agent] == loc_from:
        if agent == 'robot':
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



gtpyhop.declare_actions(transfer, grasp, release, wait_empty_box)