import gtpyhop


def pick(state, agent, obj, loc_to, traj_client):
    if agent in state.agents and obj in state.objects and not state.holding[agent]:
        return [('transfer', agent, state.at[agent], state.at[obj], traj_client), 
                ('grasp', agent, obj, traj_client), 
                ('transfer', agent, state.at[obj], loc_to, traj_client)]

def exchange(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents and state.holding[agent1] == obj:
        if obj == 'screwdriver':
            return[('grasp', agent2, obj, traj_client), 
                ('release', agent1, obj, traj_client)]
        elif obj == 'box':
            return[('grasp', agent2, obj, traj_client), 
                   ('wait_empty_box', traj_client), 
                ('transfer', agent1, 'exchange point', 'table', traj_client),
                ('release', agent1, obj, traj_client),
                ('transfer', agent1, 'table', 'X', traj_client)]

def receive(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and not state.holding[agent]:
        return [('transfer', agent, state.at[agent], 'exchange point', traj_client)]

def handover(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents:
        return [('pick', agent1, obj, 'exchange point', traj_client), 
               ('receive', agent2, obj, traj_client), 
               ('exchange', agent1, agent2, obj, traj_client)]

def pick_and_place(state, agent, loc_to, traj_client):
    if state.selected_object and agent in state.agents and loc_to in state.locations:
        return [('pick', agent, state.selected_object, loc_to, traj_client), 
                ('release', agent, state.selected_object, traj_client)]

def deliver_objects(state, agent, obj_list, traj_client):
    if agent in state.agents and set(obj_list) <= state.objects and not state.holding[agent]:
        return [('check_available_obj', obj_list, traj_client), ('process_available_objects', agent, traj_client)]

def do_nothing(state, agent, traj_client):
    if state.available_objects == []:
        return []

def sub_delivery(state, agent, traj_client):
    if state.available_objects != []:
        return [('deliver_objects', agent, state.available_objects, traj_client)]


def choose_and_deliver(state, agent, traj_client):
    if state.available_objects != []:
        return [('choose_obj',), 
                ('pick_and_place', agent, 'workspace', traj_client), 
                ('continue_delivery', agent, traj_client)]

gtpyhop.declare_task_methods('pick', pick)
gtpyhop.declare_task_methods('receive', receive)
gtpyhop.declare_task_methods('exchange', exchange)

gtpyhop.declare_task_methods('handover', handover)

gtpyhop.declare_task_methods('process_available_objects', choose_and_deliver, do_nothing)
gtpyhop.declare_task_methods('continue_delivery', sub_delivery, do_nothing)
gtpyhop.declare_task_methods('pick_and_place', pick_and_place)
gtpyhop.declare_task_methods('deliver_objects', deliver_objects)