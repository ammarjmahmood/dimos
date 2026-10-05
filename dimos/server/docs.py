# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""dimos's own docs (the checkout's docs/), for Desktop: the guide to adding a robot of your own, and links into the
published docs site.

Nothing is hardcoded but the words a page is looked for by: the site and repo URLs come from mkdocs.yml, pages are
found in the docs tree by name and title, and a page's URL is where mkdocs publishes it (`a/b.md` -> `<site>/a/b/`).
The guide is answered as markdown (links made absolute, so they work outside the site) and as HTML rendered with
markdown-it (what rich, already a dimos dependency, renders with), for a page that just shows it.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any

# a page about adding your own robot: its file name or first heading matches one of these, best first
CUSTOM_ROBOT = (
    r"(custom|new|own|your)[ _-]+(robot|platform|hardware)",
    r"(adding|add|integrat\w*)[ _-]+(a[ _-]+)?(custom|new)",
    r"(custom|new)[ _-]+\w+[ _-]+(robot|arm|platform)",
)
# Desktop's links: name -> the page file names that answer it, best first
LINKS = {
    "configure_robot": ("configuration",),
    "blueprints": ("blueprints",),
    "modules": ("modules",),
    "installation": ("installation",),
    "quickstart": ("quickstart",),
    "cli": ("cli",),
}
LINK = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\s+\"[^\"]*\")?\)")


def site(dimos_dir: Path) -> tuple[str | None, str | None]:
    """(docs site URL, repo URL) from mkdocs.yml (read as text: it has tags a YAML loader refuses)."""
    try:
        text = (dimos_dir / "mkdocs.yml").read_text()
    except OSError:
        return None, None

    def value(key: str) -> str | None:
        found = re.search(rf"^{key}:\s*['\"]?([^'\"\s#]+)", text, re.MULTILINE)
        return found.group(1) if found else None

    url = value("site_url")
    return (url.rstrip("/") + "/" if url else None), value("repo_url")


def docs_dir(dimos_dir: Path) -> Path:
    return dimos_dir / "docs"


def pages(dimos_dir: Path) -> list[Path]:
    root = docs_dir(dimos_dir)
    return sorted(
        (p for p in root.rglob("*.md") if not any(part.startswith(".") for part in p.parts)),
        key=lambda p: (len(p.relative_to(root).parts), str(p)),
    )


def title(path: Path) -> str:
    try:
        for line in path.read_text().splitlines():
            if line.startswith("# "):
                return line[2:].strip()
    except OSError:
        pass
    return path.stem.replace("_", " ")


def page_url(dimos_dir: Path, path: Path) -> str | None:
    base, _ = site(dimos_dir)
    if base is None:
        return None
    relative = path.relative_to(docs_dir(dimos_dir)).with_suffix("")
    parts = list(relative.parts)
    if parts and parts[-1] in ("index", "README"):
        parts = parts[:-1]
    return base + "".join(f"{part}/" for part in parts)


def find_custom_robot(dimos_dir: Path) -> list[Path]:
    """The pages about adding a robot of your own, best match first."""
    found: list[tuple[int, int, Path]] = []
    for path in pages(dimos_dir):
        names = f"{path.stem} {title(path)}".lower()
        for rank, pattern in enumerate(CUSTOM_ROBOT):
            if re.search(pattern, names):
                found.append((rank, len(path.parts), path))
                break
    return [path for _, _, path in sorted(found)]


def absolute_links(dimos_dir: Path, path: Path, markdown: str) -> str:
    """Relative links and images made absolute: a page to its docs-site URL, `/docs/...` too (mkdocs translates those
    repo-root links), any other repo file to the repo on GitHub."""
    base, repo = site(dimos_dir)
    root = docs_dir(dimos_dir)

    def fix(match: re.Match[str]) -> str:
        opener, target, label = match.group(1), match.group(2), match.group(3) or ""
        if re.match(r"^[a-z][a-z0-9+.-]*:|^#", target):
            return match.group(0)
        file, _, anchor = target.partition("#")
        resolved = (
            (dimos_dir / file.lstrip("/")).resolve()
            if file.startswith("/")
            else (path.parent / file).resolve()
        )
        try:
            inside_docs = resolved.relative_to(root.resolve())
        except ValueError:
            inside_docs = None
        url: str | None
        if inside_docs is not None and base and resolved.suffix == ".md":
            url = page_url(dimos_dir, root / inside_docs)
        elif inside_docs is not None and base:
            url = base + inside_docs.as_posix()
        elif repo:
            try:
                url = f"{repo.rstrip('/')}/blob/main/{resolved.relative_to(dimos_dir.resolve()).as_posix()}"
            except ValueError:
                url = None
        else:
            url = None
        if url is None:
            return match.group(0)
        return f"{opener}{url}{'#' + anchor if anchor else ''}{label})"

    return LINK.sub(fix, markdown)


def render(markdown: str) -> str | None:
    try:
        from markdown_it import MarkdownIt
    except ImportError:
        return None
    return str(MarkdownIt("commonmark", {"html": False}).enable("table").render(markdown))


def custom_robot(dimos_dir: Path) -> dict[str, Any] | None:
    found = find_custom_robot(dimos_dir)
    if not found:
        return None
    path = found[0]
    markdown = absolute_links(dimos_dir, path, path.read_text())
    return {
        "title": title(path),
        "markdown": markdown,
        "html": render(markdown),
        "source_path": path.relative_to(dimos_dir).as_posix(),
        "url": page_url(dimos_dir, path),
        "others": [
            {
                "title": title(p),
                "source_path": p.relative_to(dimos_dir).as_posix(),
                "url": page_url(dimos_dir, p),
            }
            for p in found[1:]
        ],
    }


def links(dimos_dir: Path) -> dict[str, Any]:
    base, repo = site(dimos_dir)
    all_pages = pages(dimos_dir)
    answer: dict[str, Any] = {"site": base, "repo": repo}
    for name, stems in LINKS.items():
        match = next(
            (
                p
                for stem in stems
                for p in all_pages
                if p.stem == stem or (p.stem == "index" and p.parent.name == stem)
            ),
            None,
        )
        answer[name] = page_url(dimos_dir, match) if match else None
    guides = find_custom_robot(dimos_dir)
    answer["custom_robot"] = page_url(dimos_dir, guides[0]) if guides else None
    return answer
