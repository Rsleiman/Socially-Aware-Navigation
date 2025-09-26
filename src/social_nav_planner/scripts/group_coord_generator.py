import math
import yaml
import sys
import os

def create_f_formation_template(n, interpersonal_distance=1.5):
    if not (2 <= n <= 5):
        raise ValueError("Group size must be between 2 and 5")
    
    # Calculate radius for desired interpersonal distance
    radius = interpersonal_distance / (2 * math.sin(math.pi / n))
    
    agents = []
    for i in range(n):
        # Angular position (starting from top, going clockwise)
        angle = (2 * math.pi * i) / n
        
        # Position relative to (0,0)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Orientation facing inward (toward center)
        orientation = math.degrees(angle + math.pi) % 360
        
        agents.append((round(x, 2), round(y, 2), round(orientation, 1)))
    
    return {
        'centroid': (0.0, 0.0),
        'agents': agents
    }

def translate_formation(formation_template, target_centroid):
    dx = target_centroid[0] - formation_template['centroid'][0]
    dy = target_centroid[1] - formation_template['centroid'][1]
    
    translated_agents = []
    for x, y, orientation in formation_template['agents']:
        new_x = x + dx
        new_y = y + dy
        translated_agents.append((round(new_x, 2), round(new_y, 2), orientation))
    
    return {
        'centroid': target_centroid,
        'agents': translated_agents
    }

def generate_multiple_formations(group_configs):
    formations = {}
    
    for i, config in enumerate(group_configs):
        # Create template
        template = create_f_formation_template(
            n=config['n'],
            interpersonal_distance=config['distance']
        )
        
        # Translate to target position
        final_formation = translate_formation(template, config['centroid'])
        
        formations[f'group_{i+1}'] = final_formation
    
    return formations


def degrees_to_ros_heading(degrees):
    """
    Convert degrees (0-360) to ROS heading format (-π to π)
    """
    radians = math.radians(degrees)
    # Normalize to [-π, π] range
    while radians > math.pi:
        radians -= 2 * math.pi
    while radians < -math.pi:
        radians += 2 * math.pi
    return round(radians, 2)


def formations_to_yaml_dict(formations_dict):
    # Count total agents for agent list
    total_agents = sum(len(agents) for agents in formations_dict.values())
    agent_names = [f"person{i+1}" for i in range(total_agents)]
    
    # Build YAML structure
    yaml_dict = {
        'hunav_loader': {
            'ros__parameters': {
                'map': 'default',
                'publish_people': True,
                'agents': agent_names
            }
        }
    }
    
    params = yaml_dict['hunav_loader']['ros__parameters']
    
    person_counter = 1
    group_counter = 1
    
    # Process each group
    for (center_x, center_y), agents in formations_dict.items():
        for agent_x, agent_y, orientation_degrees in agents:
            person_name = f"person{person_counter}"
            
            # Convert orientation to ROS heading
            heading = degrees_to_ros_heading(orientation_degrees)
            
            # Create person configuration
            person_config = {
                'id': person_counter,
                'skin': (person_counter - 1) % 5,  # Cycle through skin types
                'group_id': group_counter,
                'max_vel': 0.5,
                'radius': 0.4,
                'behavior': {
                    'type': 1,
                    'configuration': 1,
                    'duration': 40.0,
                    'once': True,
                    'vel': 1.0,
                    'dist': 4.5,
                    'goal_force_factor': 2.0,
                    'obstacle_force_factor': 10.0,
                    'social_force_factor': 5.0,
                    'other_force_factor': 20.0
                },
                'init_pose': {
                    'x': agent_x,
                    'y': agent_y,
                    'z': 1.250000,
                    'h': heading
                },
                'goal_radius': 0.3,
                'cyclic_goals': True,
                'goals': ["g0"],
                'g0': {
                    'x': agent_x,
                    'y': agent_y,
                    'h': heading
                }
            }
            
            params[person_name] = person_config
            person_counter += 1
        
        group_counter += 1
    
    return yaml_dict


def save_formations_to_yaml(formations_dict, filename="hunav_config.yaml", output_dir=None):
    yaml_dict = formations_to_yaml_dict(formations_dict)
    
    if output_dir:
        filepath = os.path.join(output_dir, filename)
        os.makedirs(output_dir, exist_ok=True)
    else:
        filepath = filename
    
    with open(filepath, 'w') as file:
        yaml.dump(yaml_dict, file, default_flow_style=False, sort_keys=False)
    
    print(f"Saved configuration to {filepath}")
    return yaml_dict




if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python group_coord_generator.py <config_file>")
        sys.exit(1)
    
    # Get scenarios folder path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    while current_dir != '/' and os.path.basename(current_dir) != 'src':
        current_dir = os.path.dirname(current_dir)
    scenarios_dir = os.path.join(current_dir, 'hunav_gazebo_wrapper', 'scenarios')
    
    # Load config file
    config_file = sys.argv[1]
    with open(config_file, 'r') as f:
        if config_file.endswith('.yaml') or config_file.endswith('.yml'):
            config_data = yaml.safe_load(f)
        else:
            config_data = eval(f.read())
    
    # Process each filename and its configs
    for filename, configs in config_data.items():
        for config in configs:
            if isinstance(config['centroid'], list):
                config['centroid'] = tuple(config['centroid'])
        
        formations = generate_multiple_formations(configs)
        
        formations_dict = {}
        for group_id, formation in formations.items():
            centroid = formation['centroid']
            agents = formation['agents']
            formations_dict[centroid] = agents
        
        save_formations_to_yaml(formations_dict, f"{filename}.yaml", scenarios_dir)