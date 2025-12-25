# Physical AI Documentation

This repository contains the documentation for the Physical AI project, focusing on ROS 2 as the robotic nervous system.

## Project Structure

```
.
├── docs/                     # Documentation source files
│   ├── intro.md             # Introduction page
│   └── physical-ai/         # Physical AI module documentation
│       └── module-1-ros2/   # Module 1: The Robotic Nervous System (ROS 2)
│           ├── chapter-1-robotic-nervous-system.md
│           ├── chapter-2-communication-motion.md
│           └── chapter-3-code-to-body.md
├── src/                     # Source files for custom components
│   └── css/
│       └── custom.css       # Custom CSS styles
├── static/                  # Static assets
│   └── img/                 # Images and logos
├── package.json             # Project dependencies and scripts
├── docusaurus.config.js     # Docusaurus configuration
└── sidebars.js              # Navigation sidebar configuration
```

## Getting Started

### Prerequisites

- Node.js version 18 or higher
- npm or yarn package manager

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

### Local Development

```bash
npm start
# or
yarn start
```

This command starts a local development server and opens the documentation in your browser. Most changes are reflected live without having to restart the server.

### Build

```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

### Deployment

The documentation can be deployed to GitHub Pages by running:

```bash
npm run deploy
# or
yarn deploy
```

## Documentation Modules

### Module 1: The Robotic Nervous System (ROS 2)

This module introduces ROS 2 as the communication backbone for robotics applications:

1. **Chapter 1**: ROS 2 as the Robotic Nervous System - Core concepts of ROS 2 architecture
2. **Chapter 2**: Communication in Motion - Nodes, topics, and services
3. **Chapter 3**: From Code to Body - Python integration and URDF robot description

## Contributing

To contribute to this documentation:

1. Fork the repository
2. Create a new branch for your changes
3. Make your changes to the appropriate Markdown files
4. Test your changes locally using `npm start`
5. Submit a pull request

## License

This documentation is licensed under the Creative Commons Attribution 4.0 International License.