# Quickstart: Physical AI — Module 2: The Digital Twin (Gazebo & Unity)

## Getting Started

### Prerequisites
- Node.js v18 or higher
- npm or yarn package manager
- Git for version control
- Basic understanding of ROS 2 concepts (from Module 1)
- Familiarity with simulation concepts is helpful but not required

### Setup Instructions

1. **Clone the repository** (if not already done)
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies** (if not already done)
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```

4. **Open your browser**
   - Navigate to `http://localhost:3000`
   - You should see the Physical AI documentation site with Module 2 content

### Project Structure
```
docs/
├── physical-ai/
│   ├── module-1-ros2/              # Module 1: ROS 2 Fundamentals
│   │   ├── chapter-1-robotic-nervous-system.md
│   │   ├── chapter-2-communication-motion.md
│   │   └── chapter-3-code-to-body.md
│   └── module-2-digital-twin/      # Module 2: Digital Twin (Gazebo & Unity)
│       ├── chapter-1-digital-twin-reality.md
│       ├── chapter-2-physics-gazebo.md
│       └── chapter-3-perception-unity.md
├── sidebars.js
└── docusaurus.config.js
```

### Adding New Content
1. Create a new `.md` file in the appropriate module directory
2. Add frontmatter with title and description
3. Write content in Markdown format
4. Update `sidebars.js` to include the new page in navigation (if needed)

### Building for Production
```bash
npm run build
# or
yarn build
```

The built site will be in the `build/` directory and can be deployed to any static hosting service.

### Local Testing
After making changes, always test locally:
```bash
npm run serve
# or
yarn serve
```

### Simulation Environment Setup (Advanced)
For readers who want to experiment with actual Gazebo and Unity simulations:

1. **Gazebo Installation** (for physics simulation):
   - Install ROS 2 Humble Hawksbill or later
   - Install Gazebo Garden or compatible version
   - Set up ROS 2 workspace with simulation packages

2. **Unity Setup** (for perception simulation):
   - Install Unity 2022.3 LTS or later
   - Install ROS# package for Unity-ROS communication
   - Configure sensor simulation assets

Note: These advanced simulation setups are for hands-on practice and are not required to understand the documentation concepts.