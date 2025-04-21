import xml.etree.ElementTree as ET
import numpy as np

shape = input('What kind of shape is it? sphere, box, or cylinder?: ')
if shape == 'sphere':
    m = float(input('What is the mass (kg): '))
    r = float(input('What is the radius (m): '))
    I = (m*r*r)*2/5
    print(f'I_xx = {I}\nI_yy = {I}\nI_zz = {I}')
elif shape == 'box':
    m = float(input('What is the mass (kg): '))
    l = float(input('What is the length(x) (m): '))
    w = float(input('What is the width(y) (m): '))
    h = float(input('What is the height(z) (m): '))
    Ix = m*(w*w+h*h)/12
    Iy = m*(l*l+h*h)/12
    Iz = m*(l*l+w*w)/12
    print(f'I_xx = {Ix}\nI_yy = {Iy}\nI_zz = {Iz}')
elif shape == 'cylinder':    
    m = float(input('What is the mass (kg): '))
    r = float(input('What is the radius (m): '))
    h = float(input('What is the height(z) (m): '))
    I = (m*(3*r*r+h*h))/12
    Iz = m*r*r*0.5
    print(f'I_xx = {I}\nI_yy = {I}\nI_zz = {Iz}')
elif shape == 'dae':
    # Parse the .dae file
    tree = ET.parse('/home/jshipley/Documents/ros2_ws/src/ece3730/robot/models/robot/meshes/wheel.dae')
    root = tree.getroot()

    namespace = {'': 'http://www.collada.org/2005/11/COLLADASchema'}  # Add the namespace here

    position_source = root.find(".//source[@id='X1_Wheel-POSITION']", namespace)
    # Extract position data (from the float_array you printed)
    position_data = position_source.find(".//float_array", namespace).text.split()

    scale_factor_input = input("What is the scale factor? If none, type '1': ")
    if scale_factor_input.strip() == '' or scale_factor_input.lower() == 'none':
        scale_factor = 1.0
    else:
        scale_factor = float(scale_factor_input)
    # Convert the data into a NumPy array (3 values per vertex)
    vertices = scale_factor*(np.array([float(value) for value in position_data]).reshape(-1, 3))

    # Calculate the bounding box dimensions (min and max values for each axis)
    x_min, y_min, z_min = vertices.min(axis=0)
    x_max, y_max, z_max = vertices.max(axis=0)

    # Calculate width, height, depth
    width = x_max - x_min
    height = y_max - y_min
    depth = z_max - z_min

    # Calculate the radius (distance from the origin)
    distances = np.sqrt(vertices[:, 0]**2 + vertices[:, 1]**2 + vertices[:, 2]**2)
    radius = distances.max()

    # Output the results
    print(f"Width: {width}")
    print(f"Height: {height}")
    print(f"Depth: {depth}")
    print(f"Radius: {radius}")

else:
    print('Shape not accepted. Try the script again.')

