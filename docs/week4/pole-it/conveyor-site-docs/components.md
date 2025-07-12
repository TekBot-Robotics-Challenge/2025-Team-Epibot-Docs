# Components Documentation

This document provides an overview of the various components used in the conveyor-site project. Each component is described in terms of its purpose, props, state management, and usage examples.

## 1. LogoTekbot

### Description
The `LogoTekbot` component displays the Tekbot Robotics logo and optionally includes text.

### Props
- `width` (number): The width of the logo. Default is 80.
- `height` (number): The height of the logo. Default is 50.
- `showText` (boolean): Determines whether to show the text "Tekbot Robotics". Default is true.

### Usage Example
```tsx
<LogoTekbot width={100} height={60} showText={true} />
```

## 2. LogoTRC

### Description
The `LogoTRC` component displays the TRC logo.

### Props
- `width` (number): The width of the logo. Default is 300.
- `height` (number): The height of the logo. Default is 100.
- `showText` (boolean): Currently not used.

### Usage Example
```tsx
<LogoTRC width={300} height={100} />
```

## 3. CounterCard

### Description
The `CounterCard` component is used to display a counter for each color category.

### Props
- `title` (string): The title of the counter.
- `count` (number): The current count value.
- `colorClass` (string): Tailwind CSS class for text color.
- `borderClass` (string): Tailwind CSS class for border color.
- `glowClass` (string): Tailwind CSS class for glow effect.
- `bgClass` (string): Tailwind CSS class for background color.
- `icon` (ReactNode): An icon to display alongside the counter.

### Usage Example
```tsx
<CounterCard
  title="RED"
  count={data.RED}
  colorClass="text-red-400"
  borderClass="border-red-500"
  glowClass="shadow-red-500/20"
  bgClass="bg-red-500/10"
  icon={<Recycle className="h-6 w-6" />}
/>
```

## 4. KPICard

### Description
The `KPICard` component displays a Key Performance Indicator (KPI) with a title and value.

### Props
- `title` (string): The title of the KPI.
- `value` (number): The value of the KPI.
- `icon` (ReactNode): An icon to display with the KPI.
- `color` (string): The color theme for the KPI.

### Usage Example
```tsx
<KPICard
  title="Total Sorted Items"
  value={total}
  icon={<Target className="h-8 w-8" />}
  color="cyan"
/>
```

## Conclusion

This documentation provides a brief overview of the components used in the conveyor-site project. Each component is designed to be reusable and configurable, allowing for a flexible and dynamic user interface. For further details on usage and implementation, refer to the individual component files in the project.
