---
name: card-component
description: Build accessible card components with shadows, hover effects, and responsive grids. Use when user needs modern card UI patterns.
---

# Card Component Design

## Instructions
Create card components following modern best practices:

### 1. Shadow Effects and Elevation
- Use layered shadows for depth perception
- Apply `box-shadow` with multiple comma-separated values
- Match shadow intensity to card elevation level

### 2. Hover Animations
- Smooth transitions (200-300ms)
- Lift effect: `transform: translateY(-4px)`
- Shadow expansion on hover
- Focus visible for keyboard navigation

### 3. Responsive Grid Layouts
- CSS Grid or Flexbox based layouts
- `grid-template-columns: repeat(auto-fit, minmax(300px, 1fr))`
- Mobile-first breakpoints
- Gap spacing for visual breathing room

### 4. Image Optimization
- Use `srcset` for responsive images
- Lazy loading with `loading="lazy"`
- Aspect ratio preservation with `object-fit: cover`
- Define explicit width/height to prevent layout shift

### 5. Accessibility
- Proper heading hierarchy (h2-h4 within cards)
- Focus visible states with `outline` or `ring`
- `aria-label` for icon-only content
- Sufficient color contrast (4.5:1 minimum)
- Keyboard navigation support
- Screen reader announcements for interactive cards

## Example Code
```css
/* Card base with shadow */
.card {
  background: #fff;
  border-radius: 12px;
  box-shadow:
    0 1px 3px rgba(0,0,0,0.08),
    0 4px 12px rgba(0,0,0,0.05);
  transition: transform 0.2s ease, box-shadow 0.2s ease;
  overflow: hidden;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow:
    0 4px 8px rgba(0,0,0,0.1),
    0 12px 24px rgba(0,0,0,0.08);
}

/* Focus visible for accessibility */
.card:focus-visible {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

/* Responsive grid */
.card-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 1.5rem;
  padding: 1.5rem;
}

/* Image optimization */
.card-image {
  width: 100%;
  aspect-ratio: 16 / 9;
  object-fit: cover;
}
```

## HTML Structure
```html
<article class="card" tabindex="0">
  <img
    class="card-image"
    src="image.jpg"
    srcset="image-400.jpg 400w, image-800.jpg 800w"
    sizes="(max-width: 600px) 100vw, 50vw"
    loading="lazy"
    alt="Descriptive alt text"
  >
  <div class="card-content">
    <h2 class="card-title">Card Title</h2>
    <p class="card-description">Card description text.</p>
  </div>
</article>
```
