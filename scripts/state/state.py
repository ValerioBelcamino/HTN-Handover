from gtpyhop import State

state = State("state 0")

state.objects = {'screwdriver', 'screwdriver2', 'box', 'screws', 'brick1', 'brick6', 'brick4', 'brick3', 'brick2', 'brick5'}
state.box_empty = False
state.obj_properties = {'screwdriver': None, 'screwdriver2': None, 'box': state.box_empty}
state.active_arm = {'human': None, 'robot': None}

state.markerID2obj = {'0': 'brick1', '10': 'brick2', '100': 'brick3'}
# state.obj2pose = {'brick1': pose}

# for deliver_objects method
state.available_objects = []
state.selected_object = None

state.locations = {'table', 'table2', 'exchange point', 'X', 'Y', 'workspace', 'brick1_pose', 'brick2_pose', 'brick3_pose'}
state.agents = {'human', 'robot'}


state.holding = {'human': None, 'robot': None}
state.at = {'box': 'table', 
            'screwdriver': 'table', 
            'screwdriver2': 'table2', 
            'human': 'X', 
            'robot': 'Y',
            'left': 'Y',
            'right': 'Y',
            'brick1': 'brick1_pose', 
            'brick2': 'brick2_pose', 
            'brick3': 'brick3_pose', 
            'brick6': 'brick2', 
            'brick4': 'table', 
            'brick5': 'table'}
