# Data Model: Physical AI — UI Upgrade & Visual Experience

**Feature**: 005-physical-ai-ui-upgrade
**Created**: 2025-12-26
**Input**: spec.md, plan.md, research.md

## Overview

This document defines the UI component entities and data structures for the Physical AI book UI upgrade. These entities represent the visual and structural elements that will be enhanced to create a professional and eye-catching interface.

## Color System

### Base Colors (CSS Variables)
```css
--ifm-color-primary: #2563eb;          /* Primary blue */
--ifm-color-primary-dark: #1d4ed8;     /* Darker primary */
--ifm-color-primary-darker: #1e40af;   /* Even darker */
--ifm-color-primary-darkest: #1e3a8a;  /* Darkest primary */

--ifm-color-secondary: #64748b;        /* Secondary gray */
--ifm-color-success: #10b981;          /* Success green */
--ifm-color-info: #3b82f6;             /* Info blue */
--ifm-color-warning: #f59e0b;          /* Warning amber */
--ifm-color-danger: #ef4444;           /* Danger red */
```

### Module Accent Colors
```css
--module-ros-accent: #3b82f6;          /* ROS 2: Blue */
--module-digital-twin-accent: #10b981; /* Digital Twin: Green */
--module-ai-brain-accent: #8b5cf6;     /* AI Brain: Purple */
--module-vla-accent: #f59e0b;          /* VLA: Orange */

--module-ros-light: #dbeafe;           /* ROS light */
--module-digital-twin-light: #d1fae5;  /* Digital Twin light */
--module-ai-brain-light: #e9d5ff;      /* AI Brain light */
--module-vla-light: #fef3c7;           /* VLA light */
```

### Dark Mode Colors
```css
--ifm-color-primary: #3b82f6;          /* Primary blue (dark) */
--ifm-color-primary-dark: #60a5fa;     /* Darker primary (dark) */
--ifm-background-color: #0f172a;       /* Dark background */
--ifm-background-surface-color: #1e293b; /* Surface background */
--ifm-font-color-base: #f1f5f9;        /* Light text */
```

## Typography System

### Font Stack
```css
--ifm-font-family-base: 'Inter', system-ui, -apple-system, 'Segoe UI', sans-serif;
--ifm-font-family-monospace: 'Fira Code', 'Monaco', 'Consolas', monospace;
```

### Font Sizes (Responsive Scale)
```css
--ifm-font-size-small: 0.875rem;       /* 14px */
--ifm-font-size-base: 1rem;            /* 16px */
--ifm-font-size-large: 1.125rem;       /* 18px */
--ifm-font-size-xl: 1.25rem;           /* 20px */
--ifm-font-size-2xl: 1.5rem;           /* 24px */
--ifm-font-size-3xl: 1.875rem;         /* 30px */
--ifm-font-size-4xl: 2.25rem;          /* 36px */
--ifm-font-size-5xl: 3rem;             /* 48px */
--ifm-font-size-6xl: 3.75rem;          /* 60px */
```

### Font Weights
```css
--ifm-font-weight-light: 300;
--ifm-font-weight-normal: 400;
--ifm-font-weight-medium: 500;
--ifm-font-weight-semibold: 600;
--ifm-font-weight-bold: 700;
```

### Line Heights
```css
--ifm-line-height-base: 1.65;
--ifm-line-height-computed: 1.5;
```

## Spacing System

### Spacing Scale (based on 8px base unit)
```css
--ifm-spacing-xs: 0.5rem;              /* 8px */
--ifm-spacing-sm: 0.75rem;             /* 12px */
--ifm-spacing: 1rem;                   /* 16px */
--ifm-spacing-md: 1.25rem;             /* 20px */
--ifm-spacing-lg: 1.5rem;              /* 24px */
--ifm-spacing-xl: 2rem;                /* 32px */
--ifm-spacing-2xl: 3rem;               /* 48px */
--ifm-spacing-3xl: 4rem;               /* 64px */
```

## Component Entities

### Callout Components

#### Standard Callouts
- **Info Callout**: Blue-themed, for general information
  - CSS class: `.alert--info`
  - Background: `var(--ifm-color-info)` with opacity
  - Border: left accent border in info color
  - Icon: ℹ️ (information icon)

- **Tip Callout**: Green-themed, for helpful suggestions
  - CSS class: `.alert--success`
  - Background: `var(--ifm-color-success)` with opacity
  - Border: left accent border in success color
  - Icon: 💡 (lightbulb icon)

- **Note Callout**: Purple-themed, for important notes
  - CSS class: `.alert--secondary`
  - Background: `var(--ifm-color-secondary)` with opacity
  - Border: left accent border in secondary color
  - Icon: 📝 (memo icon)

- **Caution Callout**: Amber-themed, for warnings
  - CSS class: `.alert--warning`
  - Background: `var(--ifm-color-warning)` with opacity
  - Border: left accent border in warning color
  - Icon: ⚠️ (warning icon)

#### Custom Callouts
- **Insight Callout**: Yellow-themed, for deep insights
  - CSS class: `.callout--insight`
  - Background: `var(--module-vla-light)`
  - Border: left accent border in VLA accent color
  - Icon: 💡 (lightbulb icon)

- **Mental Model Callout**: Blue-themed, for conceptual understanding
  - CSS class: `.callout--mental-model`
  - Background: `var(--module-ros-light)`
  - Border: left accent border in ROS accent color
  - Icon: 🧠 (brain icon)

- **System Flow Callout**: Green-themed, for process diagrams
  - CSS class: `.callout--system-flow`
  - Background: `var(--module-digital-twin-light)`
  - Border: left accent border in Digital Twin accent color
  - Icon: ⚙️ (gear icon)

- **Forward Link Callout**: Purple-themed, for future connections
  - CSS class: `.callout--forward-link`
  - Background: `var(--module-ai-brain-light)`
  - Border: left accent border in AI Brain accent color
  - Icon: 🚀 (rocket icon)

### Navigation Components

#### Sidebar Navigation
- **Module Categories**: Grouped by Physical AI modules
  - Header: Module name with accent color
  - Icon: Module-specific icon
  - Collapsible: Chapter lists within each module

#### Breadcrumb Navigation
- **Module Path**: Shows current module and chapter hierarchy
- **Visual Style**: Subtle, non-intrusive styling
- **Links**: Each level is clickable to navigate up the hierarchy

#### Previous/Next Navigation
- **Position**: Fixed at bottom of content
- **Visual Cues**: Clear directional indicators
- **Module Context**: Shows which module the next/previous item belongs to

### Layout Components

#### Content Container
- **Max Width**: 800px for optimal reading line length
- **Padding**: Consistent spacing on all sides
- **Background**: Clean, readable background color

#### Code Blocks
- **Styling**: Enhanced syntax highlighting
- **Background**: Subtle background color
- **Font**: Monospace with improved readability
- **Copy Button**: Integrated copy functionality

#### Tables
- **Styling**: Enhanced visual hierarchy
- **Borders**: Subtle borders for clarity
- **Alternating Rows**: Light background for alternate rows

## Module Identity Entities

### ROS 2 Module
- **Primary Color**: Blue (#3b82f6)
- **Secondary Color**: Light blue (#dbeafe)
- **Accent Elements**: Headings, links, and navigation items in ROS sections
- **Icon**: 🤖 (robot icon) or specific ROS branding

### Digital Twin Module
- **Primary Color**: Green (#10b981)
- **Secondary Color**: Light green (#d1fae5)
- **Accent Elements**: Headings, links, and navigation items in Digital Twin sections
- **Icon**: 🧊 (cube icon) or twin/digital representation

### AI Brain Module
- **Primary Color**: Purple (#8b5cf6)
- **Secondary Color**: Light purple (#e9d5ff)
- **Accent Elements**: Headings, links, and navigation items in AI Brain sections
- **Icon**: 🧠 (brain icon)

### VLA Module
- **Primary Color**: Orange (#f59e0b)
- **Secondary Color**: Light orange (#fef3c7)
- **Accent Elements**: Headings, links, and navigation items in VLA sections
- **Icon**: 🗣️ (speech bubble) or VLA representation

## Responsive Design Entities

### Breakpoints
```css
--ifm-container-width: 1140px;
--ifm-container-width-xl: 1320px;
--ifm-breakpoint-xs: 0;
--ifm-breakpoint-sm: 576px;
--ifm-breakpoint-md: 768px;
--ifm-breakpoint-lg: 996px;
--ifm-breakpoint-xl: 1200px;
--ifm-breakpoint-xxl: 1400px;
```

### Responsive Typography
- **Mobile**: Scaled-down font sizes for better readability
- **Tablet**: Medium font sizes with increased line spacing
- **Desktop**: Full font sizes with optimal line length

### Responsive Spacing
- **Mobile**: Reduced spacing for compact layout
- **Tablet**: Balanced spacing for readability
- **Desktop**: Generous spacing for enhanced visual appeal

## Accessibility Entities

### Color Contrast Ratios
- **Text on Background**: Minimum 4.5:1 for normal text, 3:1 for large text
- **Interface Elements**: Minimum 3:1 for non-text elements
- **Focus Indicators**: High contrast focus rings for keyboard navigation

### Semantic HTML Structure
- **Headings**: Proper H1-H6 hierarchy
- **Landmarks**: Main, nav, aside, header, footer elements
- **Lists**: Properly structured ordered and unordered lists

### ARIA Attributes
- **Roles**: Proper ARIA roles for complex components
- **Labels**: Accessible names for interactive elements
- **States**: Current state information for dynamic content

## Future Integration Hooks

### AI Citation Marking
- **CSS Classes**: Reserved classes for citation highlighting
- **Data Attributes**: Hooks for AI-generated citations
- **Styling**: Subtle visual indicators for citations

### RAG Chatbot Space
- **Container**: Reserved space for chatbot interface
- **Trigger**: Button or element to activate chatbot
- **Position**: Non-intrusive placement in UI

### Annotation System
- **Markers**: Hooks for content annotations
- **Tooltips**: Support for AI-generated explanations
- **Visual Style**: Consistent with overall UI design