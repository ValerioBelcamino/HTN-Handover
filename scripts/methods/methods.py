import gtpyhop


def pick(state, agent, obj, traj_client):
    if agent in state.agents and obj in state.objects and not state.holding[agent]:
        return [('transfer', agent, state.at[agent], state.at[obj], traj_client), 
                ('grasp', agent, obj, traj_client), 
                ('transfer', agent, state.at[obj], 'exchange point', traj_client)]

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
        return[('pick', agent1, obj, traj_client), 
               ('receive', agent2, obj, traj_client), 
               ('exchange', agent1, agent2, obj, traj_client)]

gtpyhop.declare_task_methods('pick', pick)
gtpyhop.declare_task_methods('receive', receive)
gtpyhop.declare_task_methods('exchange', exchange)

gtpyhop.declare_task_methods('handover', handover)