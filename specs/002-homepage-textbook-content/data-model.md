# Data Model: Docusaurus Homepage & Textbook Content Entities

This document describes the structure of key Docusaurus files and content conventions relevant to the "Homepage UI Design + Textbook Content Creation" specification. These entities define how the homepage is structured and how textbook content is managed.

## 1. Homepage Files (`frontend/src/pages/index.tsx`)

The main homepage is a React component located at `frontend/src/pages/index.tsx`. It typically uses Docusaurus-provided components and standard React practices.

### Key Sections and Their Structure:

-   **React Component (`HomePage`)**: The default export of `index.tsx`.
    -   **Layout Component**: Uses `<Layout>` from Docusaurus for consistent header, footer, and SEO.
    -   **`Head` Component**: For SEO metadata (`title`, `description`).
    -   **Hero Section**: A prominent section typically at the top, containing a title, tagline, and call-to-action button(s).
        -   `<h2>` or `<h1>` for title.
        -   `<p>` for tagline.
        -   `<Link>` components for navigation (e.g., "Get Started" to `/docs/intro`).
    -   **Feature Section**: Displays key features or modules, often using a grid layout.
        -   Typically renders components like `HomepageFeatures` (`frontend/src/components/HomepageFeatures/index.tsx`).

### Validation Rules:
-   The homepage MUST be a React component.
-   It MUST use Docusaurus's `<Layout>` component for consistent theming.
-   It MUST include a clear title and a call-to-action linking to the textbook documentation.
-   It MUST be simple and readable, avoiding complex UI elements.

## 2. Homepage Features Component (`frontend/src/components/HomepageFeatures/index.tsx`)

This is a reusable React component for displaying features on the homepage.

### Key Sections and Their Structure:

-   **`FeatureItem` Interface**: Defines the structure for each feature.
    -   `title` (string): Title of the feature.
    -   `Svg` (React.ComponentType<React.ComponentProps<'svg'>>): SVG icon for the feature.
    -   `description` (JSX.Element): Description of the feature.
-   **Features Array**: An array of `FeatureItem` objects.
-   **React Component**: Renders the `Features` array, typically in a grid.

### Validation Rules:
-   Features should be concise and relevant to the project's purpose.
-   Descriptions should be readable and not overly long.

## 3. Textbook Content Files (`frontend/docs/**/*.md`)

Each chapter is a Markdown (`.md`) file. The structure is heavily influenced by Docusaurus's content plugin for docs.

### Key Sections and Their Structure (within Markdown files):

-   **Frontmatter (YAML)**: At the top of each `.md` file.
    -   `sidebar_position` (number): Order in the sidebar.
    -   `title` (string): Title of the chapter.
    -   `sidebar_label` (string): Label displayed in the sidebar (optional, defaults to `title`).
    -   `id` (string): Unique identifier for the document (optional, defaults to filename).
-   **Content (Markdown)**: The body of the chapter.
    -   Headings (`#`, `##`, etc.) for structure.
    -   Paragraphs, lists, code blocks.
    -   **External Links**: Standard Markdown `[label](url)` syntax.
    -   **Images/Diagrams**: Standard Markdown `![alt text](path/to/image.png)` syntax.
        -   Paths can be absolute (e.g., `/img/image.png` from `frontend/static/img/`) or relative.
    -   **Tables**: Standard Markdown table syntax.

### Validation Rules:
-   Each chapter MUST contain valid YAML frontmatter.
-   Each chapter MUST be a Markdown file (`.md`).
-   Each chapter MUST be a minimum of 150 lines in length (excluding frontmatter and blank lines).
-   Content MUST be detailed and technical.
-   Content SHOULD include external links, images/diagrams, and tables where appropriate.
-   Existing content MUST NOT be overwritten; new content can only be added or extended (by appending).

## 4. `frontend/static/` Directory

Stores static assets like images, logos, and favicons that are globally accessible or used across multiple content pages.

### Key Structure:
-   `frontend/static/img/`: Contains image files (e.g., `logo.png`, `book.png`, `ai.png`).

### Validation Rules:
-   Images referenced from Markdown (`/img/image.png`) MUST exist in this directory.
