document.addEventListener('DOMContentLoaded', () => {
    const year = document.querySelector('#current-year');
    if (year) year.textContent = new Date().getFullYear();

    const tocLinks = [...document.querySelectorAll('.page-toc a')];
    const sections = tocLinks
        .map((link) => document.querySelector(link.getAttribute('href')))
        .filter(Boolean);

    if ('IntersectionObserver' in window && sections.length) {
        const linksById = new Map(
            tocLinks.map((link) => [link.getAttribute('href').slice(1), link])
        );

        const setActiveLink = (id) => {
            tocLinks.forEach((link) => {
                const active = link === linksById.get(id);
                link.classList.toggle('is-active', active);
                if (active) link.setAttribute('aria-current', 'location');
                else link.removeAttribute('aria-current');
            });
        };

        const observer = new IntersectionObserver((entries) => {
            const visible = entries
                .filter((entry) => entry.isIntersecting)
                .sort((a, b) => b.boundingClientRect.top - a.boundingClientRect.top);
            if (visible[0]) setActiveLink(visible[0].target.id);
        }, { rootMargin: '-18% 0px -70% 0px', threshold: 0 });

        sections.forEach((section) => observer.observe(section));
    }

    const mobileContents = document.querySelector('.mobile-toc details');
    document.querySelectorAll('.mobile-toc a').forEach((link) => {
        link.addEventListener('click', () => {
            if (mobileContents) mobileContents.open = false;
        });
    });

    const demo = document.querySelector('#project-highlight');
    if (demo && window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
        demo.removeAttribute('autoplay');
        demo.pause();
    }
});
