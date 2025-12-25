# Data Model: Physical AI — Module 1 (ROS 2) Documentation

## Documentation Entities

### Chapter
- **name**: String (e.g., "ROS 2 as the Robotic Nervous System")
- **slug**: String (URL-friendly identifier)
- **content**: Markdown text
- **metadata**: Frontmatter with title, description, keywords
- **relationships**: Contains multiple Sections

### Section
- **title**: String (section heading)
- **content**: Markdown text
- **type**: Enum (concept, example, exercise, summary)
- **relationships**: Belongs to a Chapter, contains multiple ContentBlocks

### ContentBlock
- **type**: Enum (text, code, diagram, example, exercise)
- **content**: String (Markdown or code)
- **metadata**: Language for code blocks, alt text for diagrams
- **relationships**: Belongs to a Section

### Example
- **title**: String (e.g., "Basic ROS 2 Publisher Node")
- **language**: String (e.g., "python", "bash")
- **code**: String (the actual code example)
- **explanation**: String (explanation of the example)
- **relationships**: Belongs to a Section

### Exercise
- **title**: String (exercise name)
- **description**: String (what the learner should do)
- **difficulty**: Enum (beginner, intermediate, advanced)
- **solution**: String (optional solution or guidance)
- **relationships**: Belongs to a Section

## Relationships
- Chapter contains many Sections
- Section contains many ContentBlocks
- Section may contain many Examples
- Section may contain many Exercises

## Validation Rules
- Chapter name must be unique within the module
- Chapter slug must be URL-friendly (alphanumeric, hyphens only)
- Content must follow Markdown format
- All code examples must be valid and tested
- Exercises must have clear objectives