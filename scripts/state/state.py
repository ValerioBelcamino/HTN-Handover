from gtpyhop import State

state = State("state 0")

state.objects = {'screwdriver', 'box', 'screws'}
state.box_empty = False
state.obj_properties = {'screwdriver': None, 'box': state.box_empty}

state.locations = {'table', 'exchange point', 'X', 'Y'}
state.agents = {'human', 'robot'}

state.holding = {'human': None, 'robot': None}
state.at = {'box': 'table', 'screwdriver': 'table', 'human': 'X', 'robot': 'Y'}
