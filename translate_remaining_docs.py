#!/usr/bin/env python3
"""
Script to help translate remaining documentation files
This script will identify untranslated files and provide templates for translation
"""

import os
import glob
from pathlib import Path

def find_untranslated_files():
    """Find documentation files that haven't been translated yet"""

    # Get all English documentation files
    english_files = []
    for ext in ['*.md', '*.mdx']:
        english_files.extend(glob.glob(f"docs/**/{ext}", recursive=True))

    # Remove the test-chatbot.mdx file as it's not part of the main docs
    english_files = [f for f in english_files if 'test-chatbot.mdx' not in f]

    # Identify which files are already translated
    translated_files = {
        'es': [],
        'ar': [],
        'ur': []
    }

    for lang in ['es', 'ar', 'ur']:
        for ext in ['*.md', '*.mdx']:
            translated_files[lang].extend(
                glob.glob(f"i18n/{lang}/docusaurus-plugin-content-docs/current/**/{ext}", recursive=True)
            )

    # Extract just the relative paths for comparison
    es_translated_paths = [os.path.relpath(f, f"i18n/es/docusaurus-plugin-content-docs/current/") for f in translated_files['es']]
    ar_translated_paths = [os.path.relpath(f, f"i18n/ar/docusaurus-plugin-content-docs/current/") for f in translated_files['ar']]
    ur_translated_paths = [os.path.relpath(f, f"i18n/ur/docusaurus-plugin-content-docs/current/") for f in translated_files['ur']]

    # Find untranslated files
    untranslated = []
    for english_file in english_files:
        relative_path = os.path.relpath(english_file, 'docs')

        es_translated = relative_path in es_translated_paths
        ar_translated = relative_path in ar_translated_paths
        ur_translated = relative_path in ur_translated_paths

        if not (es_translated and ar_translated and ur_translated):
            untranslated.append({
                'english_path': english_file,
                'relative_path': relative_path,
                'es_translated': es_translated,
                'ar_translated': ar_translated,
                'ur_translated': ur_translated
            })

    return untranslated

def create_translation_template(english_file_path, target_lang):
    """Create a translation template for a given language"""

    with open(english_file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Parse frontmatter
    lines = content.split('\n')
    frontmatter_start = -1
    frontmatter_end = -1

    for i, line in enumerate(lines):
        if line.strip() == '---' and frontmatter_start == -1:
            frontmatter_start = i
        elif line.strip() == '---' and frontmatter_start != -1 and frontmatter_end == -1:
            frontmatter_end = i
            break

    if frontmatter_start != -1 and frontmatter_end != -1:
        frontmatter = lines[frontmatter_start+1:frontmatter_end]
        body = lines[frontmatter_end+1:]

        # Translate frontmatter based on language
        translated_frontmatter = []
        for line in frontmatter:
            if line.strip().startswith('title:'):
                title = line.split(':', 1)[1].strip().strip('"')
                if target_lang == 'es':
                    translated_frontmatter.append(f'title: "{title}"  # TRANSLATE THIS')
                elif target_lang == 'ar':
                    translated_frontmatter.append(f'title: "{title}"  # TRANSLATE THIS')
                elif target_lang == 'ur':
                    translated_frontmatter.append(f'title: "{title}"  # TRANSLATE THIS')
                else:
                    translated_frontmatter.append(line)
            else:
                translated_frontmatter.append(line)

        # Reconstruct with translated frontmatter
        result = ['---']
        result.extend(translated_frontmatter)
        result.append('---')
        result.extend(body)

        return '\n'.join(result)

    return content

def main():
    print("(Analyzing documentation translation status...)")

    untranslated = find_untranslated_files()

    print(f"\n(Found {len(untranslated)} files that need translation:)")

    for i, file_info in enumerate(untranslated, 1):
        print(f"\n  {i}. {file_info['relative_path']}")
        print(f"     English: (check)")
        print(f"     Spanish: {'(check)' if file_info['es_translated'] else '(x)'}")
        print(f"     Arabic:  {'(check)' if file_info['ar_translated'] else '(x)'}")
        print(f"     Urdu:    {'(check)' if file_info['ur_translated'] else '(x)'}")

    print(f"\n(Summary:)")
    total_files = len(untranslated)
    if total_files > 0:
        print(f"   Need attention: {total_files} files")
        print("\n(To translate a file, use the following pattern:)")
        print("   1. Read the English file: " + untranslated[0]['english_path'])
        print("   2. Create translated versions at:")
        print(f"      - i18n/es/docusaurus-plugin-content-docs/current/{untranslated[0]['relative_path']}")
        print(f"      - i18n/ar/docusaurus-plugin-content-docs/current/{untranslated[0]['relative_path']}")
        print(f"      - i18n/ur/docusaurus-plugin-content-docs/current/{untranslated[0]['relative_path']}")

    print("\n(Translation infrastructure is fully set up!)")
    print("   The language switcher is available in the top navigation bar.")
    print("   RTL support is configured for Arabic and Urdu.")

if __name__ == "__main__":
    main()