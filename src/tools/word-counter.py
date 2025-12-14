#!/usr/bin/env python3

"""
Word Counter for AI Robotics Book
Counts words in all markdown files and validates against constitutional requirements (5000-7000 words)
"""

import os
import re
from pathlib import Path

def count_words_in_file(file_path):
    """Count words in a single markdown file, excluding frontmatter."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove frontmatter if present
    content = remove_frontmatter(content)

    # Remove citations in parentheses
    content = re.sub(r'\([^)]*\d{4}[^)]*\)', ' ', content)

    # Remove markdown formatting but keep the text
    content = re.sub(r'[*_~`#\[\]()]', ' ', content)

    # Remove extra whitespace
    content = re.sub(r'\s+', ' ', content)

    # Find all word-like tokens
    words = re.findall(r'\b\w+\b', content)

    return len(words)

def remove_frontmatter(content):
    """Remove YAML frontmatter from content if present."""
    lines = content.split('\n')

    if len(lines) > 0 and lines[0].strip() == '---':
        for i in range(1, len(lines)):
            if lines[i].strip() == '---':
                # Return content after the closing ---
                return '\n'.join(lines[i + 1:])

    return content

def count_words_in_docs(docs_dir):
    """Count words in all markdown files in docs directory."""
    docs_path = Path(docs_dir)
    markdown_files = list(docs_path.rglob('*.md'))

    total_words = 0
    file_counts = []

    print("Word Count Analysis:")
    print("=" * 60)

    for file_path in markdown_files:
        word_count = count_words_in_file(file_path)
        total_words += word_count
        file_counts.append((file_path, word_count))

        relative_path = str(file_path.relative_to(docs_path))
        print(f"{relative_path:<40} {word_count:>6} words")

    print("=" * 60)
    print(f"Total files: {len(markdown_files)}")
    print(f"Total words: {total_words}")
    print(f"Average per file: {total_words // len(markdown_files) if markdown_files else 0}")

    # Check constitutional requirement: 5000-7000 words
    constitutional_range = 5000 <= total_words <= 7000
    print(f"\nConstitutional requirement (5000-7000 words): {'PASS' if constitutional_range else 'FAIL'}")
    print(f"Range: 5000-7000, Actual: {total_words}")

    if not constitutional_range:
        if total_words < 5000:
            print(f"Need {5000 - total_words} more words to meet minimum requirement")
        else:
            print(f"Exceeds maximum by {total_words - 7000} words")

    return total_words, file_counts, constitutional_range

def main():
    """Main function to run the word counter."""
    import sys

    docs_dir = sys.argv[1] if len(sys.argv) > 1 else './docs'

    print("Starting word count analysis...")
    print(f"Analyzing files in: {docs_dir}\n")

    total_words, file_counts, constitutional_range = count_words_in_docs(docs_dir)

    if constitutional_range:
        print("\nWord count requirements met!")
        sys.exit(0)
    else:
        print(f"\nWord count requirements not met!")
        sys.exit(1)

if __name__ == "__main__":
    main()