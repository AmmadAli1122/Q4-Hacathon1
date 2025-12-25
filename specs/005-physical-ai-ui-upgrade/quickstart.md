# Quickstart: Physical AI — UI Upgrade & Visual Experience

**Feature**: 005-physical-ai-ui-upgrade
**Created**: 2025-12-26
**Input**: spec.md, plan.md, research.md, data-model.md

## Getting Started with UI Development

This quickstart guide provides the essential steps to begin developing the UI upgrade for the Physical AI book. Follow these steps to set up your development environment and begin implementing the visual enhancements.

## Prerequisites

### System Requirements
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control
- A modern code editor (VS Code recommended)

### Project Setup
1. Clone the repository (if not already done):
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

3. Verify the current Docusaurus site runs:
   ```bash
   npm run start
   # or
   yarn start
   ```

## Development Environment Setup

### Local Development Server
1. Start the development server:
   ```bash
   npm run start
   ```

2. Open your browser to `http://localhost:3000`

3. The site will automatically reload when you make changes to CSS or content files

### File Structure for UI Changes
```
src/
├── css/
│   ├── custom.css           # Main custom styles
│   ├── modules.css          # Module-specific styling
│   ├── typography.css       # Typography system
│   └── callouts.css         # Enhanced callout styling
└── theme/
    ├── MDXComponents.js     # Custom MDX components
    └── Navbar.js            # Enhanced navigation
```

## Key CSS Variables to Start With

### Color Variables
```css
:root {
  /* Primary colors */
  --ifm-color-primary: #2563eb;
  --module-ros-accent: #3b82f6;
  --module-digital-twin-accent: #10b981;
  --module-ai-brain-accent: #8b5cf6;
  --module-vla-accent: #f59e0b;

  /* Spacing */
  --ifm-spacing: 1rem;
  --ifm-spacing-lg: 1.5rem;
  --ifm-spacing-xl: 2rem;

  /* Typography */
  --ifm-font-family-base: 'Inter', system-ui, sans-serif;
  --ifm-font-size-base: 1rem;
  --ifm-line-height-base: 1.65;
}
```

## First UI Enhancement Tasks

### 1. Typography Improvements
1. Open `src/css/typography.css`
2. Implement the base font stack and sizing
3. Test readability with different content types

### 2. Module Accent Colors
1. Open `src/css/modules.css`
2. Implement module-specific accent colors
3. Apply to headings and navigation elements

### 3. Callout Styling
1. Open `src/css/callouts.css`
2. Implement enhanced styling for all callout types
3. Test with different content scenarios

## Testing Your Changes

### Visual Testing
1. Navigate through all modules in both light and dark mode
2. Verify consistent typography across all content
3. Check that module accent colors are applied correctly
4. Test callout visibility and readability

### Accessibility Testing
1. Use keyboard navigation to test all interactive elements
2. Verify sufficient color contrast ratios (use a11y tools)
3. Test with screen readers if available
4. Check responsive behavior on different screen sizes

### Performance Testing
1. Monitor page load times
2. Verify no significant performance degradation
3. Check bundle size impact

## Common Development Commands

### Development
```bash
# Start development server
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Testing Commands
```bash
# Check for broken links
npm run docusaurus write-heading-ids
npm run build
```

## UI Development Workflow

### 1. CSS Custom Properties Approach
- Use CSS variables for consistent theming
- Define variables in `src/css/custom.css`
- Apply variables throughout other CSS files
- Maintain consistent color and spacing systems

### 2. Module Identity Implementation
- Apply module-specific accent colors to headings
- Style navigation elements by module
- Implement module-specific callout styling
- Maintain consistent layout across modules

### 3. Responsive Design
- Test on different screen sizes (mobile, tablet, desktop)
- Verify typography scales appropriately
- Check navigation works on all devices
- Ensure content remains readable

## Integration Points

### Docusaurus Configuration
- Update `docusaurus.config.js` for any new theme settings
- Modify `themeConfig` for navigation and color mode settings
- Add any new plugins if needed for UI enhancements

### Markdown Content
- Existing content should remain unchanged
- Custom components will enhance existing callouts
- Navigation will be updated automatically

## Common UI Patterns

### Enhanced Callouts
```markdown
:::note[Insight 💡]
This is an enhanced insight callout with custom styling.
:::

:::caution[Mental Model 🧠]
This callout emphasizes conceptual understanding.
:::
```

### Module-Specific Styling
- Headings in ROS module get blue accent
- Code blocks in Digital Twin get green accent
- Navigation items show module identity
- Consistent styling across all modules

## Troubleshooting

### Styles Not Applying
1. Check if CSS files are properly imported in `src/css/custom.css`
2. Verify CSS variable names match data-model.md
3. Clear browser cache and restart development server

### Dark Mode Issues
1. Verify dark mode variables are properly defined
2. Check contrast ratios in both modes
3. Test all UI elements in dark mode

### Responsive Design Problems
1. Use browser dev tools to test different screen sizes
2. Verify CSS media queries are correctly implemented
3. Check that content remains readable on all devices

## Next Steps

1. Implement the core typography system
2. Add module accent colors
3. Enhance callout styling
4. Improve navigation elements
5. Test accessibility compliance
6. Verify responsive behavior
7. Optimize performance