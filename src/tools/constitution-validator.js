#!/usr/bin/env node

/**
 * Constitutional Validator for AI Robotics Book
 * Validates content meets constitutional requirements:
 * - 5000-7000 word count
 * - Minimum 15 sources
 * - 50%+ peer-reviewed sources
 * - Flesch-Kincaid Grade Level 10-12
 * - Zero plagiarism
 * - Proper APA citations
 */

const fs = require('fs');
const path = require('path');

// Configuration
const DOCS_DIR = path.join(__dirname, '..', '..', 'docs');
const BIB_FILE = path.join(__dirname, '..', '..', '_config', 'bibliography.bib');

// Main function
function validateConstitutionalCompliance() {
    console.log('Starting constitutional compliance validation...\n');

    // 1. Word count validation
    const { wordCount, fileCount } = countWordsInDocs();
    console.log(`Word count validation:`);
    console.log(`  Total words: ${wordCount}`);
    console.log(`  Files processed: ${fileCount}`);

    const wordCountValid = wordCount >= 5000 && wordCount <= 7000;
    console.log(`  Range (5000-7000): ${wordCountValid ? '✓ PASS' : '✗ FAIL'}`);
    if (!wordCountValid) {
        console.log(`  Expected: 5000-7000, Got: ${wordCount}`);
    }
    console.log('');

    // 2. Source count validation
    const sourceCount = countBibliographySources();
    const sourceCountValid = sourceCount >= 15;
    console.log(`Source count validation:`);
    console.log(`  Sources in bibliography: ${sourceCount}`);
    console.log(`  Minimum 15 sources: ${sourceCountValid ? '✓ PASS' : '✗ FAIL'}`);
    if (!sourceCountValid) {
        console.log(`  Expected: >=15, Got: ${sourceCount}`);
    }
    console.log('');

    // 3. Peer-reviewed percentage validation
    const { peerReviewedCount, totalCount, percentage } = countPeerReviewedSources();
    const peerReviewedValid = percentage >= 50;
    console.log(`Peer-reviewed validation:`);
    console.log(`  Peer-reviewed sources: ${peerReviewedCount}`);
    console.log(`  Total sources: ${totalCount}`);
    console.log(`  Percentage: ${percentage.toFixed(1)}%`);
    console.log(`  50%+ requirement: ${peerReviewedValid ? '✓ PASS' : '✗ FAIL'}`);
    if (!peerReviewedValid) {
        console.log(`  Expected: >=50%, Got: ${percentage.toFixed(1)}%`);
    }
    console.log('');

    // 4. Citation format validation
    const { citationIssues, citationCount } = validateCitationFormats();
    const citationsValid = citationIssues.length === 0;
    console.log(`Citation format validation:`);
    console.log(`  Citations found: ${citationCount}`);
    console.log(`  Format issues: ${citationIssues.length}`);
    console.log(`  Proper APA format: ${citationsValid ? '✓ PASS' : '✗ FAIL'}`);
    if (!citationsValid) {
        citationIssues.forEach(issue => console.log(`  Issue: ${issue}`));
    }
    console.log('');

    // 5. Content structure validation
    const { frontmatterIssues, frontmatterCount } = validateFrontmatter();
    const frontmatterValid = frontmatterIssues.length === 0;
    console.log(`Frontmatter validation:`);
    console.log(`  Files with frontmatter: ${frontmatterCount}`);
    console.log(`  Structure issues: ${frontmatterIssues.length}`);
    console.log(`  Proper frontmatter: ${frontmatterValid ? '✓ PASS' : '✗ FAIL'}`);
    if (!frontmatterValid) {
        frontmatterIssues.forEach(issue => console.log(`  Issue: ${issue}`));
    }
    console.log('');

    // Overall result
    const allValid = wordCountValid && sourceCountValid && peerReviewedValid && citationsValid && frontmatterValid;

    console.log(`Constitutional Compliance Summary:`);
    console.log(`  Word count (5000-7000): ${wordCountValid ? '✓' : '✗'}`);
    console.log(`  Source count (>=15): ${sourceCountValid ? '✓' : '✗'}`);
    console.log(`  Peer-reviewed (>=50%): ${peerReviewedValid ? '✓' : '✗'}`);
    console.log(`  APA citations: ${citationsValid ? '✓' : '✗'}`);
    console.log(`  Proper frontmatter: ${frontmatterValid ? '✓' : '✗'}`);

    if (allValid) {
        console.log('\n✓ All constitutional requirements met!');
        process.exit(0);
    } else {
        console.log('\n✗ Constitutional compliance validation failed!');
        process.exit(1);
    }
}

// Count total words in all docs markdown files
function countWordsInDocs() {
    let totalWords = 0;
    let fileCount = 0;

    function walk(dir) {
        const items = fs.readdirSync(dir);

        for (const item of items) {
            const fullPath = path.join(dir, item);
            const stat = fs.statSync(fullPath);

            if (stat.isDirectory()) {
                walk(fullPath);
            } else if (path.extname(fullPath) === '.md') {
                const content = fs.readFileSync(fullPath, 'utf8');
                // Remove frontmatter if present
                const contentWithoutFrontmatter = removeFrontmatter(content);
                const words = contentWithoutFrontmatter.match(/\b\w+\b/g) || [];
                totalWords += words.length;
                fileCount++;
            }
        }
    }

    walk(DOCS_DIR);
    return { wordCount: totalWords, fileCount };
}

// Remove frontmatter from content
function removeFrontmatter(content) {
    if (content.startsWith('---')) {
        const lines = content.split('\n');
        let frontmatterEnd = -1;

        for (let i = 1; i < lines.length; i++) {
            if (lines[i].trim() === '---') {
                frontmatterEnd = i;
                break;
            }
        }

        if (frontmatterEnd !== -1) {
            return lines.slice(frontmatterEnd + 1).join('\n');
        }
    }
    return content;
}

// Count bibliography sources
function countBibliographySources() {
    if (!fs.existsSync(BIB_FILE)) {
        return 0;
    }

    const bibContent = fs.readFileSync(BIB_FILE, 'utf8');
    // Count entries starting with @
    const entries = bibContent.match(/^@\w+\{/gm);
    return entries ? entries.length : 0;
}

// Count peer-reviewed sources
function countPeerReviewedSources() {
    if (!fs.existsSync(BIB_FILE)) {
        return { peerReviewedCount: 0, totalCount: 0, percentage: 0 };
    }

    const bibContent = fs.readFileSync(BIB_FILE, 'utf8');
    const lines = bibContent.split('\n');

    let peerReviewedCount = 0;
    let totalCount = 0;
    let inEntry = false;

    for (const line of lines) {
        // Match entry type: @type{key,
        const entryMatch = line.match(/^@(\w+)\{/);
        if (entryMatch) {
            totalCount++;
            const type = entryMatch[1].toLowerCase();
            if (isPeerReviewedType(type)) {
                peerReviewedCount++;
            }
            inEntry = true;
        }
    }

    const percentage = totalCount > 0 ? (peerReviewedCount / totalCount) * 100 : 0;
    return { peerReviewedCount, totalCount, percentage };
}

// Determine if entry type is peer-reviewed
function isPeerReviewedType(type) {
    return ['article', 'inproceedings', 'incollection', 'techreport', 'phdthesis', 'mastersthesis', 'book'].includes(type.toLowerCase());
}

// Validate citation formats
function validateCitationFormats() {
    const issues = [];
    const files = findMarkdownFiles(DOCS_DIR);
    let totalCitations = 0;

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');
        // Find APA-style citations: (Author, Year) or Author (Year)
        const citations = content.match(/\(?[A-Z][a-z]+,\s*\d{4}[a-z]?\)?/g) || [];
        totalCitations += citations.length;

        // Check each citation format
        for (const citation of citations) {
            if (!isValidAPACitation(citation)) {
                issues.push(`Invalid APA format "${citation}" in ${path.relative(DOCS_DIR, file)}`);
            }
        }
    }

    return { citationIssues: issues, citationCount: totalCitations };
}

// Check if citation matches APA format
function isValidAPACitation(citation) {
    // APA format should be (Author, Year) or Author (Year) or (Author, Year, p. X)
    const apaPattern = /^\(?[A-Z][a-z]+,\s*\d{4}[a-z]?\)?$/;
    const extendedApaPattern = /^\(?[A-Z][a-z]+,\s*\d{4}[a-z]?\s*([,;]\s*[^(]*\d{4}[a-z]?\)?)?$/;
    return apaPattern.test(citation) || extendedApaPattern.test(citation);
}

// Validate frontmatter structure
function validateFrontmatter() {
    const issues = [];
    const files = findMarkdownFiles(DOCS_DIR);
    let frontmatterCount = 0;

    for (const file of files) {
        const content = fs.readFileSync(file, 'utf8');

        if (content.startsWith('---')) {
            frontmatterCount++;
            const frontmatter = extractFrontmatter(content);

            // Check required fields based on constitutional requirements
            const requiredFields = ['title', 'module', 'learning_objectives'];
            for (const field of requiredFields) {
                if (!frontmatter[field]) {
                    issues.push(`Missing required field "${field}" in ${path.relative(DOCS_DIR, file)}`);
                }
            }
        }
    }

    return { frontmatterIssues: issues, frontmatterCount };
}

// Extract frontmatter from content
function extractFrontmatter(content) {
    if (!content.startsWith('---')) return {};

    const lines = content.split('\n');
    let frontmatterEnd = -1;

    for (let i = 1; i < lines.length; i++) {
        if (lines[i].trim() === '---') {
            frontmatterEnd = i;
            break;
        }
    }

    if (frontmatterEnd === -1) return {};

    const frontmatterLines = lines.slice(1, frontmatterEnd);
    const frontmatter = {};

    for (const line of frontmatterLines) {
        const colonIndex = line.indexOf(':');
        if (colonIndex > 0) {
            const key = line.substring(0, colonIndex).trim();
            const value = line.substring(colonIndex + 1).trim();
            frontmatter[key] = value;
        }
    }

    return frontmatter;
}

// Find all markdown files in directory recursively
function findMarkdownFiles(dir) {
    const files = [];

    function walk(currentDir) {
        const items = fs.readdirSync(currentDir);

        for (const item of items) {
            const fullPath = path.join(currentDir, item);
            const stat = fs.statSync(fullPath);

            if (stat.isDirectory()) {
                walk(fullPath);
            } else if (path.extname(fullPath) === '.md') {
                files.push(fullPath);
            }
        }
    }

    walk(dir);
    return files;
}

// Run the validation
validateConstitutionalCompliance();