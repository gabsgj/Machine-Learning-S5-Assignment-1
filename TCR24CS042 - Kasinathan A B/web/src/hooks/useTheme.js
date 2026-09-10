/**
 * useTheme — manages dark/light theme with persistence.
 * - Reads localStorage on mount; falls back to prefers-color-scheme
 * - Sets data-theme attribute on <html>
 * - Persists choice to localStorage
 */
import { useState, useEffect, useCallback } from 'react';

function getInitialTheme() {
  if (typeof window === 'undefined') return 'dark';
  const stored = localStorage.getItem('theme');
  if (stored === 'dark' || stored === 'light') return stored;
  return window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
}

export function useTheme() {
  const [theme, setTheme] = useState(getInitialTheme);

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
    localStorage.setItem('theme', theme);
  }, [theme]);

  const toggle = useCallback(() => {
    setTheme(prev => (prev === 'dark' ? 'light' : 'dark'));
  }, []);

  return { theme, toggle, isDark: theme === 'dark' };
}
