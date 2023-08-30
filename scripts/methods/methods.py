import gtpyhop


def pick(state, agent, obj, loc_to, traj_client):
    if agent in state.agents and obj in state.objects and not state.holding[agent]:
        return [('choose_arm', obj, agent),
                ('transfer', agent, state.at[obj], traj_client), 
                ('grasp', agent, obj, traj_client), 
                ('transfer', agent, loc_to, traj_client)]

# def tuck_arm(state, agent, obj, loc_to, traj_client):
#     if agent in state.agents and obj in state.objects and not state.holding[agent]:
#         return [('choose_arm', obj, agent),
#                 ('transfer', agent, state.at[obj], traj_client)]

def exchange(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents and state.holding[agent1] == obj:
        if obj == 'screwdriver' or obj == 'screwdriver2':
            return[('grasp', agent2, obj, traj_client), 
                ('wait_tool_pulling', agent1, traj_client),
                ('release', agent1, obj, traj_client),
                ('reset_active_arm', agent1)]
        elif 'box' in obj:
            return[('grasp', agent2, obj, traj_client), 
                   ('wait_empty_box', obj, traj_client), 
                ('transfer', agent1, state.boxes_home_pose[obj], traj_client),
                ('release', agent1, obj, traj_client),
                # ('transfer', agent1, 'X', traj_client),
                ('reset_active_arm', agent1)]

def receive(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects:
    #and not state.holding[agent]:
        return [('transfer', agent, 'exchange point', traj_client)]

def handover(state, agent1, agent2, obj, traj_client):
    if agent1 in state.agents and agent2 in state.agents:
        return [('pick', agent1, obj, 'exchange point', traj_client), 
               ('receive', agent2, obj, traj_client), 
               ('exchange', agent1, agent2, obj, traj_client)]

def pick_and_place(state, agent, loc_to, traj_client):
    if state.selected_object and agent in state.agents and loc_to in state.locations:
        return [('pick', agent, state.selected_object, loc_to, traj_client), 
                ('release', agent, state.selected_object, traj_client),
                ('reset_active_arm', agent),
                ('reset_selected_object',)]

def deliver_objects(state, agent, obj_list, traj_client):
    if agent in state.agents and set(obj_list) <= state.objects and not state.holding[agent]:
        return [
                # ('tuck_arm', agent, 'dummy_tuck_objL', 'tuck_positionL', traj_client),
                # ('tuck_arm', agent, 'dummy_tuck_objR', 'tuck_positionR', traj_client),
                # ('tuck_arms', agent, traj_client),
                ('check_available_obj', obj_list, traj_client), 
                ('process_available_objects', obj_list, agent, traj_client)]

def do_nothing(state, obj_list, agent, traj_client):
    if state.available_objects == []:
        return []

def sub_delivery(state, obj_list, agent, traj_client):
    if state.available_objects != []:
        return [('deliver_objects', agent, obj_list, traj_client)]


def choose_and_deliver(state, obj_list, agent, traj_client):
    if state.available_objects != []:
        return [('choose_obj',), 
                ('pick_and_place', agent, 'workspace', traj_client), 
                ('continue_delivery', obj_list, agent, traj_client)]
    
def assembly_chair(state, client):
    if state.goal_object == 'chair':
        return  [
                # ('wait_idle', client),
                # ('tuck_arms', 'robot', client),
                ('handover', 'robot', 'human', 'box', client),
                ('handover', 'robot', 'human', 'box2', client),
                ('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3', 'brick4'], client),
                ('deliver_objects', 'robot', ['brick5', 'brick6'], client),
                ('handover', 'robot', 'human', 'screwdriver', client),
                ('handover', 'robot', 'human', 'box3', client),
                ]

def assembly_bottle_holder(state, client):
    if state.goal_object == 'bottle_holder':
        return  [
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['shelf1'], client),
                ('deliver_objects', 'robot', ['brick1', 'brick2', 'brick3', 'brick4', 'brick5', 'brick6', 'brick7', 'brick8'], client), # bricks 5-8 are round bricks
                ('deliver_objects', 'robot', ['shelf2', 'shelf3'], client),
                ('handover', 'robot', 'human', 'box2', client),
                ('handover', 'robot', 'human', 'screwdriver', client),
                ('handover', 'robot', 'human', 'box3', client),
                ]

def assembly_child_chair(state, client):
    if state.goal_object == 'child_chair':
        return [
                ('handover', 'robot', 'human', 'box', client),
                ('deliver_objects', 'robot', ['brick1', 'brick2'], client), # brick2 is the back of the chair
                ('deliver_objects', 'robot', ['brick3'], client), # brick3 is the seat of the chair
                ('deliver_objects', 'robot', ['brick4', 'brick5'], client), # these are the sides of the chair
                ('handover', 'robot', 'human', 'box2', client),
                ('handover', 'robot', 'human', 'screwdriver', client),
                ]

def assembly_paper_holder(state, client):
    if state.goal_object == 'paper_holder':
        return [
                ('deliver_objects', 'robot', ['brick1'], client), # brick1 is 1st side of the paper holder
                ('deliver_objects', 'robot', ['brick2', 'brick3', 'brick4', 'brick5', 'brick6'], client),
                ('handover', 'robot', 'human', 'box', client),
                ('handover', 'robot', 'human', 'screwdriver', client),
                ('deliver_objects', 'robot', ['brick7'], client), # brick7 is 2nd side of the paper holder
                ('handover', 'robot', 'human', 'box2', client),
                ('handover', 'robot', 'human', 'box3', client),
                ('deliver_objects', 'robot', ['brick8'], client), # brick8 is the paper holder brick
                ]

def loop(state, traj_client):
    if True:
        return [
            ('define_goal',), 
            ('wait_idle', traj_client),
            ('assembly', traj_client), 
            ('reset_goal',), 
            ('wait_idle', traj_client), 
            ('loop', traj_client)]

gtpyhop.declare_task_methods('pick', pick)
gtpyhop.declare_task_methods('receive', receive)
gtpyhop.declare_task_methods('exchange', exchange)
# gtpyhop.declare_task_methods('tuck_arm', tuck_arm)

gtpyhop.declare_task_methods('handover', handover)

gtpyhop.declare_task_methods('process_available_objects', choose_and_deliver, do_nothing)
gtpyhop.declare_task_methods('continue_delivery', sub_delivery, do_nothing)
gtpyhop.declare_task_methods('pick_and_place', pick_and_place)
gtpyhop.declare_task_methods('deliver_objects', deliver_objects)
# gtpyhop.declare_task_methods('assembly_chair', assembly_chair)
gtpyhop.declare_task_methods('assembly', assembly_chair, assembly_bottle_holder, assembly_child_chair, assembly_paper_holder)
gtpyhop.declare_task_methods('loop', loop)
