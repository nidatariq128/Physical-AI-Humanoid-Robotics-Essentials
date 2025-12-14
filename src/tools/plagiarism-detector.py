#!/usr/bin/env python3

"""
Plagiarism Detector for AI Robotics Book
Basic implementation to check for potential plagiarism by comparing text similarity
"""

import os
import re
from pathlib import Path
from collections import Counter

def preprocess_text(text):
    """Preprocess text by removing citations, markdown formatting, and normalizing."""
    # Remove frontmatter if present
    text = remove_frontmatter(text)

    # Remove citations in parentheses (Author, Year) or [Author, Year]
    text = re.sub(r'\([^)]*\d{4}[^)]*\)', ' ', text)
    text = re.sub(r'\[[^\]]*\d{4}[^\]]*\]', ' ', text)

    # Remove markdown formatting
    text = re.sub(r'[*_~`#\[\]()]', ' ', text)

    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text)

    # Convert to lowercase
    text = text.lower().strip()

    return text

def remove_frontmatter(content):
    """Remove YAML frontmatter from content if present."""
    lines = content.split('\n')

    if len(lines) > 0 and lines[0].strip() == '---':
        for i in range(1, len(lines)):
            if lines[i].strip() == '---':
                # Return content after the closing ---
                return '\n'.join(lines[i + 1:])

    return content

def get_ngrams(text, n=3):
    """Generate n-grams from text."""
    words = text.split()
    if len(words) < n:
        return []

    ngrams = []
    for i in range(len(words) - n + 1):
        ngram = ' '.join(words[i:i+n])
        ngrams.append(ngram)

    return ngrams

def calculate_similarity(text1, text2, n=3):
    """Calculate similarity between two texts based on n-gram overlap."""
    ngrams1 = set(get_ngrams(text1, n))
    ngrams2 = set(get_ngrams(text2, n))

    if not ngrams1 and not ngrams2:
        return 0.0

    intersection = len(ngrams1.intersection(ngrams2))
    union = len(ngrams1.union(ngrams2))

    if union == 0:
        return 0.0

    return intersection / union

def check_for_plagiarism_in_docs(docs_dir, similarity_threshold=0.8):
    """Check for potential plagiarism across all markdown files in docs directory."""
    docs_path = Path(docs_dir)
    markdown_files = list(docs_path.rglob('*.md'))

    if len(markdown_files) < 2:
        print("Need at least 2 files to check for plagiarism.")
        return True  # Consider it clean if there's only one file

    print("Plagiarism Detection Analysis:")
    print("=" * 50)

    file_contents = {}
    for file_path in markdown_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            processed_content = preprocess_text(content)
            file_contents[file_path] = processed_content

    issues = []

    # Compare each file with every other file
    for i, file1 in enumerate(markdown_files):
        for j, file2 in enumerate(markdown_files):
            if i >= j:  # Avoid comparing file with itself and duplicate comparisons
                continue

            similarity = calculate_similarity(file_contents[file1], file_contents[file2])

            if similarity > similarity_threshold:
                issues.append({
                    'file1': file1.relative_to(docs_path),
                    'file2': file2.relative_to(docs_path),
                    'similarity': similarity
                })
                print(f"Potential issue: {file1.name} vs {file2.name} - {similarity:.2f}")

    print(f"\nFiles analyzed: {len(markdown_files)}")
    print(f"Potential plagiarism issues found: {len(issues)}")

    if issues:
        print("\nDetailed issues:")
        for issue in issues:
            print(f"  {issue['file1']} <-> {issue['file2']}: {issue['similarity']:.2f}")
        return False  # Plagiarism detected
    else:
        print("  No potential plagiarism issues detected.")
        return True  # No plagiarism detected

def main():
    """Main function to run the plagiarism detector."""
    import sys

    docs_dir = sys.argv[1] if len(sys.argv) > 1 else './docs'

    print("Starting plagiarism detection...")
    print(f"Analyzing files in: {docs_dir}\n")

    clean = check_for_plagiarism_in_docs(docs_dir)

    if clean:
        print("\n✓ No plagiarism issues detected!")
        sys.exit(0)
    else:
        print(f"\n✗ Potential plagiarism issues detected!")
        sys.exit(1)

if __name__ == "__main__":
    main()