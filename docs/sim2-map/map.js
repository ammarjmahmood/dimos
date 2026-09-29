/* Reading aids only. No runtime connection or simulation controls. */
const $ = selector => document.querySelector(selector);
const escape = value => String(value).replace(/[&<>"']/g, char => ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'}[char]));
const source = window.SIM2_SOURCE;
for (const container of document.querySelectorAll('[data-sources]')) {
  container.innerHTML = container.dataset.sources.split(',').map(key => {
    const item = source.excerpts[key];
    return `<details class="source-excerpt"><summary>${escape(item.symbol)}</summary><a class="source-link" target="_blank" rel="noopener" href="../../${item.path}">${escape(item.path)}:${item.line}</a><div class="source-code">${item.code.split('\n').map((line, i) => `<div class="source-line"><span class="line-number">${item.line + i}</span><span class="line-text">${escape(line)}</span></div>`).join('')}</div></details>`;
  }).join('');
}

const chapters = [...document.querySelectorAll('section[id]')];
let scheduled = false;
function currentChapter() {
  const current = [...chapters].reverse().find(section => section.getBoundingClientRect().top <= innerHeight * 0.3) || chapters[0];
  document.querySelectorAll('.contents a').forEach(link => {
    const active = link.hash === '#' + current.id;
    link.classList.toggle('active', active);
    if (active) link.setAttribute('aria-current', 'location'); else link.removeAttribute('aria-current');
  });
  scheduled = false;
}
addEventListener('scroll', () => { if (!scheduled) { scheduled = true; requestAnimationFrame(currentChapter); } }, {passive:true});
addEventListener('resize', currentChapter);
currentChapter();

const storageKey = 'dimos-sim2-architecture-notes';
const noteKey = 'loop-walkthrough';
let notes = {};
try { notes = JSON.parse(localStorage.getItem(storageKey) || '{}'); }
catch { $('#note-status').textContent = 'Storage unavailable. Export notes before closing.'; }
$('#review-note').value = notes[noteKey] || '';
$('#review-note').addEventListener('input', event => {
  notes[noteKey] = event.target.value;
  try { localStorage.setItem(storageKey, JSON.stringify(notes)); $('#note-status').textContent = 'Saved in this browser'; }
  catch { $('#note-status').textContent = 'Not persisted. Export notes before closing.'; }
});
$('#export-notes').addEventListener('click', () => {
  const text = '# sim2 Loop Review\n\n' + Object.entries(notes).filter(([, value]) => String(value).trim()).map(([key, value]) => `## ${key}\n\n${value}\n`).join('\n');
  const url = URL.createObjectURL(new Blob([text], {type:'text/markdown'}));
  const anchor = document.createElement('a'); anchor.href = url; anchor.download = 'sim2-loop-review.md'; anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
$('#source-status').textContent = `Local source excerpts captured ${new Date(source.generated).toLocaleDateString('en-GB')}. This describes the current code; it is not a live simulator trace.`;
