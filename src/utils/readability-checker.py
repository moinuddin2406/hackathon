#!/usr/bin/env python3
"""
Readability Checker for Markdown Files
Validates Flesch-Kincaid grade level and word count for educational content.
"""

import textstat
import sys
import re


def extract_content(file_path):
    """Extract text content from markdown, excluding code blocks and front matter."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove YAML front matter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove code blocks
    content = re.sub(r'```[\s\S]*?```', '', content)

    # Remove inline code
    content = re.sub(r'`[^`]+`', '', content)

    # Remove HTML comments
    content = re.sub(r'<!--[\s\S]*?-->', '', content)

    # Remove URLs
    content = re.sub(r'https?://[^\s]+', '', content)

    # Remove markdown image syntax
    content = re.sub(r'!\[.*?\]\(.*?\)', '', content)

    # Remove markdown link syntax but keep link text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

    return content


def check_readability(file_path, min_words=1200, max_words=1600, min_grade=10, max_grade=12):
    """Check readability metrics for a markdown file."""
    try:
        text = extract_content(file_path)

        # Calculate metrics
        grade_level = textstat.flesch_kincaid_grade(text)
        word_count = textstat.lexicon_count(text, removepunct=True)

        print(f"\n[FILE] {file_path}")
        print(f"{'=' * 60}")
        print(f"[METRICS] Word Count: {word_count}")
        print(f"[METRICS] Flesch-Kincaid Grade Level: {grade_level:.1f}")
        print(f"{'=' * 60}\n")

        # Validation
        word_count_valid = min_words <= word_count <= max_words
        grade_valid = min_grade <= grade_level <= max_grade

        if word_count_valid:
            print(f"[PASS] Word count within range ({min_words}-{max_words})")
        else:
            print(f"[FAIL] Word count outside range (expected {min_words}-{max_words}, got {word_count})")
            if word_count < min_words:
                print(f"   -> Need {min_words - word_count} more words")
            else:
                print(f"   -> Need to remove {word_count - max_words} words")

        if grade_valid:
            print(f"[PASS] Readability grade within range ({min_grade}-{max_grade})")
        else:
            print(f"[FAIL] Readability grade outside range (expected {min_grade}-{max_grade}, got {grade_level:.1f})")
            if grade_level < min_grade:
                print(f"   -> Content is too simple; add technical depth")
            else:
                print(f"   -> Content is too complex; simplify language")

        print()

        return word_count_valid and grade_valid

    except FileNotFoundError:
        print(f"[ERROR] File not found: {file_path}")
        return False
    except Exception as e:
        print(f"[ERROR] Error processing file: {e}")
        return False


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python readability-checker.py <markdown_file> [--min-words N] [--max-words N] [--min-grade N] [--max-grade N]")
        print("\nExample: python readability-checker.py docs/chapter1.md")
        print("Example: python readability-checker.py docs/chapter1.md --min-words 1000 --max-words 2000")
        sys.exit(1)

    file_path = sys.argv[1]

    # Parse optional arguments
    min_words = 1200
    max_words = 1600
    min_grade = 10
    max_grade = 12

    i = 2
    while i < len(sys.argv):
        if sys.argv[i] == '--min-words' and i + 1 < len(sys.argv):
            min_words = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--max-words' and i + 1 < len(sys.argv):
            max_words = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--min-grade' and i + 1 < len(sys.argv):
            min_grade = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--max-grade' and i + 1 < len(sys.argv):
            max_grade = float(sys.argv[i + 1])
            i += 2
        else:
            print(f"Warning: Unknown argument '{sys.argv[i]}'")
            i += 1

    success = check_readability(file_path, min_words, max_words, min_grade, max_grade)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
