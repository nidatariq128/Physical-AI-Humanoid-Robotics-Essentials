#!/usr/bin/env python3
"""
Simple readability analyzer for the AI Robotics Book
Calculates Flesch-Kincaid Grade Level to ensure content meets Grade Level 10-12 requirement
"""
import re
import os
from pathlib import Path

def count_syllables(word):
    """Count syllables in a word using basic rules"""
    word = word.lower().strip()
    if len(word) <= 3:
        return 1

    # Remove trailing e
    if word.endswith('e'):
        word = word[:-1]

    vowels = 'aeiouy'
    syllable_count = 0
    prev_was_vowel = False

    for char in word:
        is_vowel = char in vowels
        if is_vowel and not prev_was_vowel:
            syllable_count += 1
        prev_was_vowel = is_vowel

    if syllable_count == 0:
        syllable_count = 1

    return syllable_count

def analyze_readability(text):
    """Calculate Flesch-Kincaid Grade Level for text"""
    # Remove markdown formatting for analysis
    text = re.sub(r'[#*\[\]_`\-]', ' ', text)
    text = re.sub(r'\n+', ' ', text)  # Replace newlines with spaces

    # Split into sentences (basic approach)
    sentences = re.split(r'[.!?]+', text)
    sentences = [s.strip() for s in sentences if s.strip()]

    # Split into words
    words = re.findall(r'\b\w+\b', text)

    # Count syllables
    syllable_count = sum(count_syllables(word) for word in words)

    # Calculate metrics
    num_sentences = len(sentences)
    num_words = len(words)

    if num_sentences == 0 or num_words == 0:
        return 0, 0, 0

    # Flesch-Kincaid Grade Level formula
    # 0.39 * (total words / total sentences) + 11.8 * (total syllables / total words) - 15.59
    avg_words_per_sentence = num_words / num_sentences
    avg_syllables_per_word = syllable_count / num_words

    grade_level = 0.39 * avg_words_per_sentence + 11.8 * avg_syllables_per_word - 15.59

    return grade_level, num_words, syllable_count

def analyze_file(filepath):
    """Analyze readability of a single file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Find the main content (after frontmatter)
        lines = content.split('\n')
        main_content = []
        in_frontmatter = False
        frontmatter_end = -1

        for i, line in enumerate(lines):
            if line.strip() == '---':
                if not in_frontmatter:
                    in_frontmatter = True
                else:
                    frontmatter_end = i
                    in_frontmatter = False
                    break
            elif not in_frontmatter and frontmatter_end == -1:
                main_content = lines  # No frontmatter found, use all content
                break

        if frontmatter_end >= 0:
            main_content = lines[frontmatter_end + 1:]

        main_text = '\n'.join(main_content)

        grade_level, word_count, syllable_count = analyze_readability(main_text)
        return grade_level, word_count, syllable_count
    except Exception as e:
        print(f"Error analyzing {filepath}: {e}")
        return 0, 0, 0

def main():
    docs_dir = Path("docs")
    if not docs_dir.exists():
        print("docs directory not found")
        return

    total_grade_level = 0
    file_count = 0

    print("Readability Analysis Results:")
    print("=" * 50)

    for md_file in docs_dir.rglob("*.md"):
        grade_level, word_count, syllable_count = analyze_file(md_file)
        print(f"{md_file.name}: Grade Level {grade_level:.2f} ({word_count} words)")

        if word_count > 0:  # Only count files with content
            total_grade_level += grade_level
            file_count += 1

    if file_count > 0:
        avg_grade_level = total_grade_level / file_count
        print("=" * 50)
        print(f"Average Grade Level: {avg_grade_level:.2f}")
        print(f"Target Range: 10-12")

        if 10 <= avg_grade_level <= 12:
            print("✓ Average readability meets target range (Grade Level 10-12)")
        else:
            print("✗ Average readability does not meet target range")
            if avg_grade_level < 10:
                print("  Content may be too simple")
            else:
                print("  Content may be too complex")
    else:
        print("No files found to analyze")

if __name__ == "__main__":
    main()