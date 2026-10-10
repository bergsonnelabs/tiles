#!/usr/bin/env python3
"""
Parse tile driver tile driver .h files into structured JSON for the docs system.

Extracts from Doxygen-style comments:
  - File-level brief and description (specs, hardware notes, quick-start code)
  - Defines (I2C addresses, register map, constants)
  - Enums with member descriptions
  - Function declarations with @brief, @param, @return, @note

Usage:
    python3 parse_driver_header.py /path/to/tile_drive_h.h
    python3 parse_driver_header.py /path/to/inc/ --tiles tile_sense_i_9 tile_drive_h tile_drive_p tile_power_l_1t
    python3 parse_driver_header.py /path/to/inc/ --all
"""

import re
import json
import sys
import os
from pathlib import Path


# ---- Console encoding ----------------------------------------------------
# Windows Python falls back to the legacy ANSI code page (cp1252) whenever
# stdout isn't a real console — which is exactly the case under make, where it
# is a pipe. Any non-ASCII in our progress output then raises
# UnicodeEncodeError and takes the build down with it. Force UTF-8 on the
# streams we own; errors="replace" keeps a terminal that genuinely cannot
# render a character from crashing over it.
for _stream in (sys.stdout, sys.stderr):
    try:
        if (getattr(_stream, "encoding", "") or "").lower().replace("-", "") != "utf8":
            _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, OSError, ValueError):
        pass


def strip_comment_prefix(lines):
    """Remove leading ' * ' or ' *' from doc-comment body lines."""
    result = []
    for line in lines:
        line = line.rstrip()
        # Match ' * text', ' *', or ' */'
        m = re.match(r'^\s*\*\s?(.*)$', line)
        if m:
            result.append(m.group(1))
        else:
            result.append(line)
    return result


def parse_usage(body):
    """`@studio usage` — a driver-authored note on how to use the tile from
    Studio (what init leaves running, what to call, what to avoid). Written
    in the file-level block like `@studio unsupported`:

        @studio usage
          Starts measuring at 25 Hz as soon as init returns, ...

    The text may start on the tag line and continue on the following lines
    until a blank line or the next @-directive; lines are joined with single
    spaces. Surrounding double quotes are dropped. Returns None when the
    header has no (or an empty) usage tag; only the first one counts.
    """
    for i, line in enumerate(body):
        m = re.match(r'@studio\s+usage\b(.*)', line.strip())
        if not m:
            continue
        parts = [m.group(1).strip()] if m.group(1).strip() else []
        for cont in body[i + 1:]:
            if cont.strip() == '' or cont.lstrip().startswith('@'):
                break
            parts.append(cont.strip())
        text = ' '.join(parts).strip()
        if len(text) >= 2 and text[0] == '"' and text[-1] == '"':
            text = text[1:-1].strip()
        return text or None
    return None


def parse_file_header(text):
    """Extract the file-level doc comment (first /** ... */ block)."""
    m = re.match(r'\s*/\*\*\s*\n(.*?)\*/\s*\n', text, re.DOTALL)
    if not m:
        return {}

    body = strip_comment_prefix(m.group(1).split('\n'))

    header = {}
    # @file
    for line in body:
        fm = re.match(r'@file\s+(.*)', line)
        if fm:
            header['file'] = fm.group(1).strip()
            break

    # @brief
    brief_lines = []
    in_brief = False
    for line in body:
        bm = re.match(r'@brief\s+(.*)', line)
        if bm:
            brief_lines.append(bm.group(1).strip())
            in_brief = True
            continue
        if in_brief:
            if line.strip() == '' or line.startswith('@'):
                in_brief = False
            else:
                brief_lines.append(line.strip())
    header['brief'] = ' '.join(brief_lines) if brief_lines else ''

    # Description: everything between @brief and the first section marker,
    # excluding @file, @brief, @code blocks
    desc_lines = []
    code_blocks = []
    in_desc = False
    in_code = False
    current_code_label = ''
    current_code = []

    for i, line in enumerate(body):
        if re.match(r'@file\s', line):
            continue
        if re.match(r'@brief\s', line):
            in_desc = True
            continue  # skip the brief line itself
        if not in_desc:
            continue

        if '@code' in line:
            in_code = True
            # Check if previous non-empty line is a label
            for j in range(i - 1, -1, -1):
                if body[j].strip():
                    current_code_label = body[j].strip().rstrip(':')
                    # Remove that label from desc_lines if it was added
                    if desc_lines and desc_lines[-1].strip().rstrip(':') == current_code_label:
                        desc_lines.pop()
                    break
            current_code = []
            continue
        if '@endcode' in line:
            in_code = False
            code_blocks.append({
                'label': current_code_label,
                'code': '\n'.join(current_code)
            })
            current_code_label = ''
            continue
        if in_code:
            current_code.append(line.rstrip())
            continue

        desc_lines.append(line)

    # Trim leading/trailing blank lines from description
    while desc_lines and desc_lines[0].strip() == '':
        desc_lines.pop(0)
    while desc_lines and desc_lines[-1].strip() == '':
        desc_lines.pop()

    header['description'] = '\n'.join(desc_lines)
    header['code_examples'] = code_blocks

    # @studio tile — driver opts in to the Studio palette. Presence of
    # this tag (anywhere in the file-level block) flips `studio_ready`,
    # which the website uses to show a Studio badge in the docs index
    # and to render the exposed/internal function split. Absent on
    # drivers that haven't been annotated yet (most tiles, as of A4d-3).
    header['studio_ready'] = any('@studio tile' in line for line in body)

    # @studio usage — top-level `usage` string in both the docs manifest
    # (here) and the Studio palette manifest (gen_studio_manifest reads it
    # through this same function). Absent when the driver has none.
    usage = parse_usage(body)
    if usage:
        header['usage'] = usage

    # @studio event — declared at file scope. Captured for the tile page
    # to render an "Events" section alongside the function lists. We
    # carry only `name` and (optional) `description` here; the manifest
    # generator owns the full payload schema for codegen.
    events = []
    for line in body:
        em = re.match(r'@studio\s+event\s+name=(\S+)(.*)', line)
        if em:
            entry = {'name': em.group(1).strip()}
            tail = em.group(2)
            dm = re.search(r'description="([^"]*)"', tail)
            if dm:
                entry['description'] = dm.group(1)
            events.append(entry)
    if events:
        header['studio_events'] = events

    # @studio unsupported — chip capabilities the driver doesn't expose,
    # surfaced as "Driver gaps" rows on the docs Coverage Table. Each
    # block looks like:
    #
    #   @studio unsupported severity=<sev> category="<text>"
    #     <continuation-line-1>
    #     <continuation-line-2>
    #     ...
    #
    # Continuation lines accumulate into the brief until a blank line
    # or the next @-directive. Severity is one of common / advanced /
    # niche; category is free text. The driver-authoring recipe in
    # CLAUDE.md owns the conventions.
    unsupported = []
    i = 0
    while i < len(body):
        line = body[i]
        m = re.match(r'@studio\s+unsupported\s+(.*)', line)
        if not m:
            i += 1
            continue
        head = m.group(1)
        sev_m = re.search(r'severity=(\S+)', head)
        cat_m = re.search(r'category="([^"]*)"', head)
        if not (sev_m and cat_m):
            i += 1
            continue
        brief_lines = []
        j = i + 1
        while j < len(body):
            cont = body[j]
            if cont.strip() == '':
                break
            if cont.lstrip().startswith('@'):
                break
            brief_lines.append(cont.strip())
            j += 1
        entry = {
            'severity': sev_m.group(1).strip(),
            'category': cat_m.group(1),
            'brief': ' '.join(brief_lines),
        }
        # Optional `section=<bucket>` threads this gap into a specific
        # Coverage Table section row group. Without it, the gap renders
        # in the generic "Driver gaps" footer at the bottom of the table.
        sec_m = re.search(r'section=([A-Za-z_][A-Za-z0-9_-]*)', head)
        if sec_m:
            entry['section'] = sec_m.group(1)
        unsupported.append(entry)
        i = j
    if unsupported:
        header['studio_unsupported'] = unsupported

    return header


def parse_defines(text):
    """Extract #define constants with their inline doc comments.

    Filters out internal register map defines (*_REG_*, *_BANK_*)
    that are driver-internal. Only emits user-facing defines:
    - Instance addresses (*_I2C_ADDR_*, *_SPI_CS_*)
    - Chip identity (*_WHOAMI_*, *_CHIP_ID_*, *_DEVICE_ID_*)
    - Status masks (*_STATUS_*, *_FAULT_*)
    - Version defines (*_VERSION_*)
    """
    # Patterns for internal defines to skip
    INTERNAL_PATTERNS = re.compile(
        r'_REG_|_BANK_|_USER_CTRL'
    )

    defines = []
    # Gather section blocks between /* --- */ markers
    sections = re.split(r'/\*\s*-{10,}\s*\*/\s*\n', text)

    for section in sections:
        # Find section header
        header_match = re.match(r'/\*\s*(.*?)\s*\*/\s*\n', section.strip())
        section_name = header_match.group(1).strip() if header_match else ''

        # Skip entire register map sections
        lower = section_name.lower()
        if 'register map' in lower or 'register' in lower and 'status' not in lower:
            continue

        for m in re.finditer(
            r'(?:(?:/\*\*\s*@brief\s+(.*?)\s*\*/\s*\n)\s*)?'
            r'#define\s+(\w+)\s+(0x[0-9A-Fa-f]+|\d+)'
            r'(?:\s*/\*\*<\s*(.*?)\s*\*/)?',
            section
        ):
            name = m.group(2)
            # Skip internal register defines that slipped through
            if INTERNAL_PATTERNS.search(name):
                continue

            doc = m.group(1) or m.group(4) or ''
            defines.append({
                'name': name,
                'value': m.group(3),
                'description': doc.strip(),
                'section': section_name,
            })

    return defines


def parse_enums(text):
    """Extract typedef enums with member descriptions."""
    enums = []

    # Pattern 1: multi-line doc comment before typedef enum
    #   /** \n ... \n */ \n typedef enum { ... } name;
    pattern_multi = re.compile(
        r'/\*\*\s*\n((?:(?!\*/).)*?)\*/\s*\n\s*typedef\s+enum\s*\{(.*?)\}\s*(\w+)\s*;',
        re.DOTALL
    )

    # Pattern 2: single-line doc comment before typedef enum
    #   /** brief text */ \n typedef enum { ... } name;
    # Note: brief capture [^\n]+ prevents matching across lines (avoids
    # grabbing the file-level /** block as a single-line doc).
    pattern_single = re.compile(
        r'/\*\*\s+([^\n]+?)\s*\*/\s*\n\s*typedef\s+enum\s*\{(.*?)\}\s*(\w+)\s*;',
        re.DOTALL
    )

    def _parse_members(enum_body):
        members = []
        for mm in re.finditer(
            r'(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*,?\s*(?:/\*\*<?\s*(.*?)\s*\*/)?',
            enum_body
        ):
            members.append({
                'name': mm.group(1),
                'value': mm.group(2),
                'description': mm.group(3) or '',
            })
        return members

    seen_names = set()

    # Multi-line doc comments
    for m in pattern_multi.finditer(text):
        doc_body = strip_comment_prefix(m.group(1).split('\n'))
        enum_body = m.group(2)
        enum_name = m.group(3)

        # Extract brief
        brief = ''
        desc_lines = []
        in_brief = False
        for line in doc_body:
            bm = re.match(r'@brief\s+(.*)', line)
            if bm:
                brief = bm.group(1).strip()
                in_brief = True
                continue
            if in_brief and line.strip() and not line.startswith('@'):
                brief += ' ' + line.strip()
                continue
            in_brief = False
            if line.strip() and not line.startswith('@'):
                desc_lines.append(line)

        enums.append({
            'name': enum_name,
            'brief': brief,
            'description': '\n'.join(desc_lines).strip(),
            'members': _parse_members(enum_body),
        })
        seen_names.add(enum_name)

    # Single-line doc comments (only if not already matched by multi-line)
    for m in pattern_single.finditer(text):
        enum_name = m.group(3)
        if enum_name in seen_names:
            continue
        brief = m.group(1).strip()
        enum_body = m.group(2)

        enums.append({
            'name': enum_name,
            'brief': brief,
            'description': '',
            'members': _parse_members(enum_body),
        })
        seen_names.add(enum_name)

    return enums


def parse_structs(text):
    """Extract typedef structs with member descriptions."""
    structs = []

    # Multi-line doc comment before typedef struct
    pattern_multi = re.compile(
        r'/\*\*\s*\n((?:(?!\*/).)*?)\*/\s*\n\s*typedef\s+struct\s*\{(.*?)\}\s*(\w+)\s*;',
        re.DOTALL
    )

    # Single-line doc comment before typedef struct
    pattern_single = re.compile(
        r'/\*\*\s+([^\n]+?)\s*\*/\s*\n\s*typedef\s+struct\s*\{(.*?)\}\s*(\w+)\s*;',
        re.DOTALL
    )

    def _parse_fields(body):
        fields = []
        for fm in re.finditer(
            r'(\w[\w\s\*]+?)\s+(\w+)\s*;\s*(?:/\*\*<?\s*(.*?)\s*\*/)?',
            body
        ):
            ftype = ' '.join(fm.group(1).split())
            fname = fm.group(2)
            fdesc = fm.group(3) or ''
            fields.append({
                'type': ftype,
                'name': fname,
                'description': fdesc,
            })
        return fields

    seen_names = set()

    for m in pattern_multi.finditer(text):
        doc_body = strip_comment_prefix(m.group(1).split('\n'))
        struct_name = m.group(3)

        brief = ''
        desc_lines = []
        in_brief = False
        for line in doc_body:
            bm = re.match(r'@brief\s+(.*)', line)
            if bm:
                brief = bm.group(1).strip()
                in_brief = True
                continue
            if in_brief and line.strip() and not line.startswith('@'):
                brief += ' ' + line.strip()
                continue
            in_brief = False
            if line.strip() and not line.startswith('@'):
                desc_lines.append(line)

        structs.append({
            'name': struct_name,
            'brief': brief,
            'description': '\n'.join(desc_lines).strip(),
            'fields': _parse_fields(m.group(2)),
        })
        seen_names.add(struct_name)

    for m in pattern_single.finditer(text):
        struct_name = m.group(3)
        if struct_name in seen_names:
            continue
        structs.append({
            'name': struct_name,
            'brief': m.group(1).strip(),
            'description': '',
            'fields': _parse_fields(m.group(2)),
        })
        seen_names.add(struct_name)

    return structs


def parse_functions(text):
    """Extract function declarations with their doc comments."""
    functions = []

    # Match /** doc */ immediately followed by a function declaration.
    # Supports both multi-line and single-line doc comments:
    #   /** @brief Foo. */          (single-line)
    #   /**\n * @brief Foo.\n */    (multi-line)
    pattern = re.compile(
        r'/\*\*\s*((?:(?!\*/).)*?)\*/\s*\n'
        r'\s*([\w*]+\s+\**\s*\w+\s*\([^)]*\))\s*;',
        re.DOTALL
    )

    for m in pattern.finditer(text):
        raw = m.group(1).strip()
        # Single-line: /** @brief Foo. */ — no newlines in body
        if '\n' not in raw:
            doc_body = [raw]
        else:
            doc_body = strip_comment_prefix(raw.split('\n'))
        signature = ' '.join(m.group(2).split())  # normalize whitespace

        func = {'signature': signature, 'params': [], 'notes': []}

        # Detect @studio expose — flags this function as Studio-callable
        # from the DSL palette. Authoritative source is the manifest
        # generator (cores/tools/gen_studio_manifest.py); we read the same
        # tag here so the website can render exposed/internal splits
        # without a separate manifest fetch.
        #
        # Optional `section=<bucket>` parameter places the function into
        # an explicit Coverage Table section (init / lifecycle / runtime
        # / config / fifo / events / advanced / other). Untagged
        # functions fall through to TileDocClient's heuristic
        # categorization based on naming conventions.
        for line in doc_body:
            if '@studio expose' in line:
                func['studio_exposed'] = True
                sm = re.search(r'section=([A-Za-z_][A-Za-z0-9_-]*)', line)
                if sm:
                    func['studio_section'] = sm.group(1)
                break

        # Parse doc tags
        brief_lines = []
        desc_lines = []
        in_brief = False
        in_desc = False
        in_note = False

        for line in doc_body:
            # @brief
            bm = re.match(r'@brief\s+(.*)', line)
            if bm:
                brief_lines = [bm.group(1).strip()]
                in_brief = True
                in_desc = False
                in_note = False
                continue

            # @param
            pm = re.match(r'@param\s+(\w+)\s+(.*)', line)
            if pm:
                in_brief = False
                in_desc = False
                in_note = False
                func['params'].append({
                    'name': pm.group(1),
                    'description': pm.group(2).strip(),
                })
                continue

            # @return
            rm = re.match(r'@return\s+(.*)', line)
            if rm:
                in_brief = False
                in_desc = False
                in_note = False
                func['return'] = rm.group(1).strip()
                continue

            # @note
            nm = re.match(r'@note\s+(.*)', line)
            if nm:
                in_brief = False
                in_desc = False
                in_note = True
                func['notes'].append(nm.group(1).strip())
                continue

            # Any other @-directive (@code, @studio expose, …) ends prose
            # collection — @brief/@param/@return/@note are handled above, so
            # anything reaching here shouldn't leak into the description/notes.
            if line.strip().startswith('@'):
                in_brief = False
                in_desc = False
                in_note = False
                continue

            # Continuation lines
            if in_note:
                if line.strip() == '' or line.strip().startswith('@'):
                    in_note = False
                else:
                    # Append to the last note
                    func['notes'][-1] += ' ' + line.strip()
                continue

            if in_brief:
                if line.strip() == '':
                    in_brief = False
                    in_desc = True
                else:
                    brief_lines.append(line.strip())
                continue

            if in_desc or (not in_brief and line.strip() and not line.startswith('@')):
                in_desc = True
                desc_lines.append(line)
                continue

        func['brief'] = ' '.join(brief_lines)
        func['description'] = '\n'.join(desc_lines).strip()

        # Extract function name from signature
        nm = re.search(r'(\w+)\s*\(', signature)
        if nm:
            func['name'] = nm.group(1)

        functions.append(func)

    return functions


def parse_header(filepath):
    """Parse a complete .h file into structured doc JSON."""
    with open(filepath, 'r') as f:
        text = f.read()

    result = parse_file_header(text)
    result['defines'] = parse_defines(text)
    result['enums'] = parse_enums(text)
    result['structs'] = parse_structs(text)
    result['functions'] = parse_functions(text)
    result['source_file'] = os.path.basename(filepath)

    # Extract driver version from defines (e.g. TILE_SENSE_I_9_VERSION_MAJOR)
    ver_major = re.search(r'#define\s+\w+_VERSION_MAJOR\s+(\d+)', text)
    ver_minor = re.search(r'#define\s+\w+_VERSION_MINOR\s+(\d+)', text)
    ver_patch = re.search(r'#define\s+\w+_VERSION_PATCH\s+(\d+)', text)
    if ver_major and ver_minor and ver_patch:
        result['version'] = f"{ver_major.group(1)}.{ver_minor.group(1)}.{ver_patch.group(1)}"
    else:
        result['version'] = '1.0.0'  # default for drivers without version defines yet

    # Extract tile family and name from filename
    # e.g. tile_sense_i_9.h -> family=Sense, variant=I.9
    basename = os.path.basename(filepath).replace('.h', '')
    if basename in ('tile_hal', 'tiles_hal'):
        result['tile_family'] = 'Platform'
        result['tile_name'] = 'HAL'
        result['display_name'] = 'Tiles HAL'
    elif basename.startswith('tile_'):
        parts = basename[5:]  # strip 'tile_'
        # Map known families
        family_map = {
            'sense': 'Sense', 'drive': 'Drive', 'power': 'Power',
            'display': 'Display', 'tak': 'Tak', 'store': 'Store',
        }
        for prefix, family in family_map.items():
            if parts.startswith(prefix + '_'):
                variant = parts[len(prefix) + 1:]
                # Convert underscores to dots for display: i_9 -> I.9
                display_variant = variant.upper().replace('_', '.')
                result['tile_family'] = family
                result['tile_name'] = display_variant
                result['display_name'] = f'{family}.{display_variant}'
                break

    return result


def main():
    if len(sys.argv) < 2:
        print("Usage: parse_driver_header.py <file_or_dir> [--tiles name1 name2] [--all]")
        sys.exit(1)

    target = sys.argv[1]

    if os.path.isfile(target):
        result = parse_header(target)
        print(json.dumps(result, indent=2))
    elif os.path.isdir(target):
        # Filter tiles
        if '--all' in sys.argv:
            files = sorted(Path(target).glob('tile_*.h'))
            # Also include tiles_hal.h
            hal = Path(target) / 'tiles_hal.h'
            if hal.exists():
                files = [hal] + list(files)
        elif '--tiles' in sys.argv:
            idx = sys.argv.index('--tiles')
            tile_names = sys.argv[idx + 1:]
            files = []
            hal = Path(target) / 'tiles_hal.h'
            if hal.exists():
                files.append(hal)
            for name in tile_names:
                f = Path(target) / f'{name}.h'
                if f.exists():
                    files.append(f)
                else:
                    print(f"Warning: {f} not found", file=sys.stderr)
        else:
            print("Specify --all or --tiles <names>", file=sys.stderr)
            sys.exit(1)

        results = []
        for f in files:
            try:
                results.append(parse_header(str(f)))
            except Exception as e:
                print(f"Error parsing {f}: {e}", file=sys.stderr)

        print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
