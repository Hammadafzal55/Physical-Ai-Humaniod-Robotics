# Research: Docusaurus Homepage UI Design & Textbook Content Creation

## Decision: Docusaurus Homepage Customization and Textbook Content Management

**Rationale**:
This feature focuses on designing a minimal, readable homepage UI and creating/extending textbook content within the existing Docusaurus framework. The primary goal is to leverage Docusaurus's native capabilities to achieve these objectives while ensuring Vercel deployability and adherence to strict UI/UX guidelines (simplicity, readability, mobile-friendliness, consistent theming).

**Alternatives Considered**:
No alternative frameworks were considered as Docusaurus is the mandated frontend technology. Research focuses on optimal utilization of Docusaurus features.

## Docusaurus Best Practices for Homepage UI Design

To achieve a minimal, readable, and mobile-friendly homepage, Docusaurus offers several customization options:

### 1. Customizing `src/pages/index.tsx`
-   The default Docusaurus homepage (`src/pages/index.tsx`) is a React component. It can be fully customized to include a hero section, feature blocks, and calls to action that guide users to the textbook content.
-   Emphasis will be on clean layout, clear typography, and minimal visual clutter, aligning with "documentation-first design."

### 2. Leveraging existing Docusaurus components
-   Docusaurus provides ready-to-use React components (e.g., `Layout`, `Head`) that can be used to build the homepage while maintaining consistency with the overall theme.
-   Existing `HomepageFeatures` (from `src/components/HomepageFeatures/index.tsx`) can be adapted or replaced.

### 3. Styling with `src/css/custom.css`
-   For fine-tuning typography and spacing, `frontend/src/css/custom.css` can be used. However, customizations will be minimal and carefully controlled to avoid breaking responsiveness or theme consistency, adhering to the "avoid custom CSS that breaks responsiveness or theming" rule.

### 4. Vercel Readiness
-   Docusaurus generates static HTML, CSS, and JavaScript files, making it inherently compatible with static site hosting platforms like Vercel.
-   The build process should produce a fully static output. Any client-side rendering should not rely on server-side logic from the Vercel deployment itself.
-   Configuration in `docusaurus.config.ts` (e.g., `baseUrl`) must be appropriate for the Vercel deployment environment.

## Textbook Content Creation & Extension Guidelines

To ensure detailed, technical chapters of at least 150 lines, including links, images, and tables, without overwriting existing content:

### 1. Markdown Files (`.md`)
-   Each chapter will be a separate Markdown file within its respective module directory (e.g., `frontend/docs/Module-1-ROS 2/new-chapter.md`).
-   Markdown's syntax for headings, lists, code blocks, links, images, and tables will be used extensively.

### 2. Frontmatter for Metadata
-   Each chapter's Markdown file should include YAML frontmatter for metadata such as `sidebar_position`, `title`, and `sidebar_label`. This helps Docusaurus organize and display content correctly.

### 3. Content Requirements
-   **Detail and Technical Depth**: Content will be thoroughly researched and presented in a technically accurate and detailed manner.
-   **Minimum Length**: Each chapter aims for a minimum of 150 lines of *meaningful* content (excluding code blocks and markdown syntax) to ensure sufficient depth.
-   **External Reference Links**: Incorporate relevant external links to academic papers, official documentation, and authoritative resources for further reading.
-   **Images and Diagrams**: Use `![]()` Markdown syntax for images. Images will be stored in `frontend/static/img/` or relative to the Markdown file. Diagrams can be embedded as images or generated using Markdown extensions if available and simple.
-   **Tables**: Utilize Markdown table syntax for presenting structured data (e.g., comparisons, lists of specifications).

### 4. Content Extension Strategy
-   **Identify Gaps**: Review existing chapters (if any) and identify areas where content can be expanded or new topics can be introduced.
-   **New Files for New Chapters**: For entirely new chapters, create new Markdown files.
-   **Append to Existing Files**: For extending existing chapters, content will be appended to the end of the existing Markdown files. This avoids accidental overwriting of prior work.

## Consistent Theming and Layout

To ensure consistent application of Light, Dark, and System theme modes across Navbar, Documentation pages, and Sidebars, and to maintain full viewport height for sidebars:

### 1. Docusaurus `themeConfig`
-   The `themeConfig` in `docusaurus.config.ts` is the central place to configure `colorMode` and ensure `respectPrefersColorScheme` is set to `true`. This ensures the site respects the user's system preferences.
-   `prism` theme settings for code blocks are also crucial for visual consistency in different modes.

### 2. Custom CSS for Layout (`src/css/custom.css`)
-   While avoiding custom CSS that breaks responsiveness, `custom.css` can be used for minor adjustments to ensure full viewport height for sidebars and visual alignment with the Navbar. This might involve adjusting `height` or `min-height` properties for relevant Docusaurus layout elements (e.g., `.docSidebarContainer`, `.tableOfContents_`).
-   Ensure color variables (`var(--ifm-color-primary)`) are used for custom styles to adapt to theme changes.

### 3. Docusaurus Native Patterns
-   Prioritize using Docusaurus's built-in theme components and styling conventions to ensure maintainability and theme consistency. Avoid deep overrides of Docusaurus's internal component styling unless absolutely necessary and thoroughly tested for responsiveness across themes.
