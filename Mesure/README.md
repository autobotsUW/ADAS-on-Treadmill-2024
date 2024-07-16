### save_topic
- **Description**: This node saves data in several topics in a CSV file.

## Topics

To allow communication between these nodes, several topics are used:

### /car_position
- **Type**: Int32 list
- **Description**: List of car positions and their dimensions.
- **Format**: `[id_1, x_1, y_1, θ_1, height_1, width_1, id_2, x_2, y_2, θ_2, height_2, width_2, ...]`

### /input_position
- **Type**: Int32 list
- **Description**: User input positions.
- **Format**: `[id_1, X_input1, Y_input1, id_2, X_input2, Y_input2, ...]`

### /command
- **Type**: Int32 list
- **Description**: Speed and angle commands for the car.
- **Format**: `[id_1, speed_1, angle_1, id_2, speed_2, angle_2, ...]`

### /obstacles_position
- **Type**: Int32 list
- **Description**: Position of obstacles.
- **Format**: `[x_1, y_1, radius_1, x_2, y_2, radius_2, ...]`
