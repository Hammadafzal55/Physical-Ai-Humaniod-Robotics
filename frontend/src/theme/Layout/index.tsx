import React, { useEffect } from 'react';
import Layout from '@theme-original/Layout';
import type { Props } from '@theme/Layout';

export default function LayoutWrapper(props: Props): JSX.Element {
  useEffect(() => {
    const handleScroll = () => {
      const winScroll = document.documentElement.scrollTop || document.body.scrollTop;
      const height = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      const scrolled = (winScroll / height) * 100;
      const progressBar = document.getElementById("scroll-progress-bar");
      if (progressBar) progressBar.style.width = scrolled + "%";
    };

    window.addEventListener("scroll", handleScroll);
    return () => window.removeEventListener("scroll", handleScroll);
  }, []);

  return (
    <>
      <div className="scroll-progress-container">
        <div className="scroll-progress-bar" id="scroll-progress-bar"></div>
      </div>
      <Layout {...props} />
    </>
  );
}
