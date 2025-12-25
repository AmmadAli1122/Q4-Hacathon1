# Data Model: Physical AI — Module 2: The Digital Twin (Gazebo & Unity)

## Documentation Entities

### Chapter
- **name**: String (e.g., "The Digital Twin — Simulating Reality")
- **slug**: String (URL-friendly identifier)
- **content**: Markdown text
- **metadata**: Frontmatter with title, description, keywords
- **relationships**: Contains multiple Sections

### Section
- **title**: String (section heading)
- **content**: Markdown text
- **type**: Enum (concept, example, exercise, simulation, tutorial)
- **relationships**: Belongs to a Chapter, contains multiple ContentBlocks

### ContentBlock
- **type**: Enum (text, code, diagram, simulation, example, exercise)
- **content**: String (Markdown or code)
- **metadata**: Language for code blocks, alt text for diagrams
- **relationships**: Belongs to a Section

### SimulationExample
- **title**: String (e.g., "Gazebo Physics Simulation Setup")
- **type**: Enum (gazebo, unity)
- **description**: String (what the simulation demonstrates)
- **prerequisites**: Array of String (required setup steps)
- **configuration**: String (configuration details)
- **relationships**: Belongs to a Section

### SensorSimulation
- **type**: Enum (lidar, depth_camera, imu, camera)
- **parameters**: Object (configuration parameters for the sensor)
- **output_format**: String (format of the sensor data)
- **ros_integration**: String (how it connects to ROS 2)
- **relationships**: Belongs to a Section

### Exercise
- **title**: String (exercise name)
- **description**: String (what the learner should do)
- **difficulty**: Enum (beginner, intermediate, advanced)
- **solution**: String (optional solution or guidance)
- **simulation_component**: Enum (gazebo, unity, sensor_simulation)
- **relationships**: Belongs to a Section

## Relationships
- Chapter contains many Sections
- Section contains many ContentBlocks
- Section may contain many SimulationExamples
- Section may contain many SensorSimulations
- Section may contain many Exercises

## Validation Rules
- Chapter name must be unique within the module
- Chapter slug must be URL-friendly (alphanumeric, hyphens only)
- Content must follow Markdown format
- All code examples must be valid and tested
- Simulation examples must include prerequisites and configuration
- Exercises must have clear objectives and difficulty levels