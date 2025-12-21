# Research: Docusaurus Multi-Module & Chapter Setup

## Decision: Docusaurus Configuration for Multi-Module & Chapter Textbook

**Rationale**:
Docusaurus is explicitly chosen as the frontend technology for building the AI-native textbook, as stated in the project's constitution. Its core strengths lie in documentation-driven websites, making it a suitable choice for a structured textbook with multiple modules and chapters. The goal is to leverage Docusaurus's built-in features for navigation, content organization, and mobile responsiveness to meet the requirements of the spec.

Docusaurus supports organizing content into categories and using sidebar configurations (`sidebars.ts`) to create hierarchical navigation. This allows for defining modules as top-level categories and chapters as sub-items within those modules. The `docusaurus.config.ts` file will be central to configuring themes, plugins, navbar, and other global settings to ensure a clean, readable, and mobile-friendly layout. Static assets like images and diagrams are handled through standard Markdown practices and by placing them in the `static` directory or relative paths within the `docs` content.

**Alternatives Considered**:
The project's constitution already mandates Docusaurus as the frontend technology. Therefore, no alternative documentation frameworks (e.g., GitBook, Sphinx, ReadTheDocs) were considered at this stage for the purpose of the research. The focus is solely on best practices and optimal configuration within the Docusaurus ecosystem to meet the specified requirements.

## Docusaurus Best Practices for Multi-Module & Chapter Setup

To achieve the desired 4 modules and 14 chapters with clean navigation and mobile-friendliness, the following Docusaurus features and practices will be utilized:

### 1. Content Organization (`docs` directory)
-   **Modules as Directories**: Each module (e.g., "Module-1-ROS 2") will be a top-level directory within the `docs` folder.
-   **Chapters as Markdown Files**: Each chapter will be a Markdown (`.md`) file within its respective module directory.
-   **`_category_.json`**: Use `_category_.json` files within each module directory to define the display name, position, and other metadata for the module in the sidebar. This ensures proper ordering and categorization.

### 2. Sidebar Configuration (`sidebars.ts`)
-   **Automatic Sidebar Generation**: Docusaurus can automatically generate sidebars based on the `docs` directory structure. This is often the simplest approach for a large, hierarchical content structure.
-   **Explicit Sidebar Definition (if needed)**: For more granular control over ordering, labels, or to mix content types, `sidebars.ts` can be explicitly defined. It will contain an array of sidebar items, where each item can be a category (module) or a document (chapter).
-   **Category Types**: Utilize Docusaurus's `category` type in `sidebars.ts` to represent modules and their nested chapters.

### 3. Global Configuration (`docusaurus.config.ts`)
-   **Navbar Configuration**: Define `navbar` items for global navigation, including links to the main documentation (the textbook).
-   **Theme Configuration**: Customize the `@docusaurus/preset-classic` theme. This includes:
    -   **Custom CSS**: Use `src/css/custom.css` for typography, spacing, and other styling adjustments to ensure a clean and readable layout.
    -   **Color Mode**: Ensure both light and dark modes are well-supported for accessibility.
    -   **Mobile Responsiveness**: The classic theme is inherently responsive, but custom CSS can further refine the mobile experience if specific layouts are required.
-   **Plugins & Presets**: Ensure the `@docusaurus/preset-classic` is configured correctly, which includes `@docusaurus/plugin-content-docs`.
-   **Base URL**: Configure `baseUrl` if the Docusaurus site is not hosted at the root of a domain (e.g., `/docs/`).

### 4. Static Assets
-   **`static` directory**: Place larger, globally used static assets (e.g., `ai.png`, `logo.png`) in the `static` directory. These can be referenced directly from the root `/` in Markdown.
-   **Relative Paths**: For images specific to a chapter, place them alongside the Markdown file and use relative paths.
-   **Image/Diagram Integration**: Use standard Markdown image syntax (`![alt text](path/to/image.png)`) for embedding images.

### 5. Mobile Responsiveness and Readability
-   The default Docusaurus theme is highly responsive. Custom CSS (`src/css/custom.css`) will be used for fine-tuning typography (font families, sizes, line heights), proper spacing (margins, padding), and ensuring optimal readability on various screen sizes. Media queries will be used for mobile-specific adjustments if necessary beyond the default theme's responsiveness.