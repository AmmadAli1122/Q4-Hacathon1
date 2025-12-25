# Quickstart: Physical AI — Module 1 (ROS 2) Documentation

## Getting Started

### Prerequisites
- Node.js v18 or higher
- npm or yarn package manager
- Git for version control

### Setup Instructions

1. **Clone the repository**
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Install dependencies**
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
   - You should see the Physical AI documentation site

### Project Structure
```
docs/
├── physical-ai/
│   └── module-1-ros2/
│       ├── chapter-1-robotic-nervous-system.md
│       ├── chapter-2-communication-motion.md
│       └── chapter-3-code-to-body.md
├── sidebars.js
└── docusaurus.config.js
```

### Adding New Content
1. Create a new `.md` file in the appropriate directory
2. Add frontmatter with title and description
3. Write content in Markdown format
4. Update `sidebars.js` to include the new page in navigation

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