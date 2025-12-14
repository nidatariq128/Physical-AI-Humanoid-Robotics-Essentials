#!/usr/bin/env python3

"""
Readability Checker for AI Robotics Book
Calculates Flesch-Kincaid Grade Level to ensure content meets 10-12 grade level requirement
"""

import os
import re
import math
from pathlib import Path

def count_syllables(word):
    """Count the number of syllables in a word."""
    word = word.lower()
    vowels = "aeiouy"
    syllable_count = 0
    prev_was_vowel = False

    for i, char in enumerate(word):
        is_vowel = char in vowels
        if is_vowel and not prev_was_vowel:
            syllable_count += 1
        prev_was_vowel = is_vowel

    # Handle silent 'e' at the end
    if word.endswith('e') and syllable_count > 1:
        syllable_count -= 1

    # Handle words that end with 'ed' but aren't pronounced as extra syllable
    if word.endswith('ed') and syllable_count > 1:
        # Some 'ed' endings don't add a syllable
        if word[-3] not in 'td':
            pass  # Don't reduce syllable count for most cases
        elif word[-3] in 'td':
            # 'ed' after 't' or 'd' sometimes adds syllable, but we'll be conservative
            pass

    # Every word has at least one syllable
    if syllable_count == 0:
        syllable_count = 1

    return syllable_count

def calculate_flesch_kincaid_grade_level(text):
    """Calculate the Flesch-Kincaid Grade Level for the given text."""
    # Remove frontmatter if present
    text = remove_frontmatter(text)

    # Clean the text
    # Remove citations in parentheses
    text = re.sub(r'\([^)]*\d{4}[^)]*\)', ' ', text)
    # Remove markdown formatting
    text = re.sub(r'[*_~`]', ' ', text)

    # Count sentences (ends with ., !, or ?)
    sentences = re.split(r'[.!?]+', text)
    sentences = [s.strip() for s in sentences if s.strip()]
    sentence_count = len(sentences)

    # Count words
    words = re.findall(r'\b\w+\b', text)
    word_count = len(words)

    # Count syllables
    syllable_count = sum(count_syllables(word) for word in words)

    if sentence_count == 0 or word_count == 0:
        return 0.0

    # Calculate Flesch-Kincaid Grade Level
    # Formula: 0.39 * (total words / total sentences) + 11.8 * (total syllables / total words) - 15.59
    avg_words_per_sentence = word_count / sentence_count
    avg_syllables_per_word = syllable_count / word_count

    grade_level = 0.39 * avg_words_per_sentence + 11.8 * avg_syllables_per_word - 15.59

    return round(grade_level, 2)

def remove_frontmatter(content):
    """Remove YAML frontmatter from content if present."""
    lines = content.split('\n')

    if len(lines) > 0 and lines[0].strip() == '---':
        for i in range(1, len(lines)):
            if lines[i].strip() == '---':
                # Return content after the closing ---
                return '\n'.join(lines[i + 1:])

    return content

def check_readability_in_file(file_path):
    """Check readability of a single file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    grade_level = calculate_flesch_kincaid_grade_level(content)
    return grade_level

def check_readability_in_docs(docs_dir):
    """Check readability of all markdown files in docs directory."""
    docs_path = Path(docs_dir)
    markdown_files = list(docs_path.rglob('*.md'))

    total_grade_level = 0
    file_count = 0
    valid_files = 0
    issues = []

    print("Readability Analysis:")
    print("=" * 50)

    for file_path in markdown_files:
        grade_level = check_readability_in_file(file_path)
        total_grade_level += grade_level
        file_count += 1

        relative_path = str(file_path.relative_to(docs_path))
        print(f"{relative_path}: Grade Level {grade_level}")

        # Check if grade level is within acceptable range (10-12)
        if 10 <= grade_level <= 12:
            valid_files += 1
        else:
            issues.append((relative_path, grade_level))

    if file_count == 0:
        print("No markdown files found to analyze.")
        return 0, True, []

    overall_grade_level = total_grade_level / file_count
    target_met = 10 <= overall_grade_level <= 12

    print("\n" + "=" * 50)
    print(f"Files analyzed: {file_count}")
    print(f"Overall average grade level: {overall_grade_level:.2f}")
    print(f"Files within target range (10-12): {valid_files}/{file_count}")
    print(f"Target grade level (10-12): {'PASS' if target_met else 'FAIL'}")

    if issues:
        print("\nFiles outside target range:")
        for file_path, grade in issues:
            status = "HIGH" if grade > 12 else "LOW"
            print(f"  {file_path}: {grade} ({status})")

    return overall_grade_level, target_met, issues

def main():
    """Main function to run the readability checker."""
    import sys

    docs_dir = sys.argv[1] if len(sys.argv) > 1 else './docs'

    print("Starting readability analysis...")
    print(f"Analyzing files in: {docs_dir}\n")

    overall_grade, target_met, issues = check_readability_in_docs(docs_dir)

    if target_met:
        print("\nReadability requirements met!")
        sys.exit(0)
    else:
        print(f"\nReadability requirements not met! Target: Grade 10-12, Actual: {overall_grade:.2f}")
        sys.exit(1)

if __name__ == "__main__":
    main()