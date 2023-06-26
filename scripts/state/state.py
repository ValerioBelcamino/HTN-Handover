from gtpyhop import State

state = State("state 0")

state.objects = {'screwdriver', 'box', 'screws', 'brick1', 'brick6', 'brick4', 'brick3', 'brick2', 'brick5'}
state.box_empty = False
state.obj_properties = {'screwdriver': None, 'box': state.box_empty}

# for deliver_objects method
state.available_objects = []
state.selected_object = None

state.locations = {'table', 'exchange point', 'X', 'Y', 'workspace'}
state.agents = {'human', 'robot'}

state.holding = {'human': None, 'robot': None}
state.at = {'box': 'table', 
            'screwdriver': 'table', 
            'human': 'X', 
            'robot': 'Y', 
            'brick1': 'table', 
            'brick6': 'table', 
            'brick4': 'table', 
            'brick3': 'table', 
            'brick2': 'table', 
            'brick5': 'table'}
