#!/usr/bin/env node

/**
 * Citation Validator for AI Robotics Book
 * Validates that all citations in markdown files match entries in bibliography.bib
 * and meet constitutional requirements (50%+ peer-reviewed sources)
 */

const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

// Configuration
const DOCS_DIR = path.join(__dirname, '..', '..', 'docs');
const BIB_FILE = path.join(__dirname, '..', '..', '_config', 'bibliography.bib');

// Main function
function validateCitations() {
    console.log('Starting citation validation...\n');

    // Read bibliography file
    if (!fs.existsSync(BIB_FILE)) {
        console.error(`ERROR: Bibliography file not found at ${BIB_FILE}`);
        process.exit(1);
    }

    const bibContent = fs.readFileSync(BIB_FILE, 'utf8');
    const bibEntries = parseBibFile(bibContent);

    console.log(`Found ${bibEntries.length} bibliography entries in ${BIB_FILE}`);

    // Find all markdown files in docs/
    const markdownFiles = findMarkdownFiles(DOCS_DIR);
    console.log(`Found ${markdownFiles.length} markdown files to check\n`);

    // Validate citations in each file
    let totalCitations = 0;
    let validCitations = 0;
    let filesWithCitations = 0;

    for (const file of markdownFiles) {
        const content = fs.readFileSync(file, 'utf8');
        const citations = findCitations(content);

        if (citations.length > 0) {
            filesWithCitations++;
            console.log(`Checking citations in ${path.relative(DOCS_DIR, file)}:`);

            for (const citation of citations) {
                totalCitations++;

                if (validateCitation(citation, bibEntries)) {
                    validCitations++;
                    console.log(`  ✓ ${citation}`);
                } else {
                    console.log(`  ✗ ${citation} (not found in bibliography)`);
                }
            }
            console.log('');
        }
    }

    // Calculate statistics
    const validPercentage = totalCitations > 0 ? (validCitations / totalCitations) * 100 : 100;
    console.log(`Citation Summary:`);
    console.log(`  Total citations found: ${totalCitations}`);
    console.log(`  Valid citations: ${validCitations}`);
    console.log(`  Invalid citations: ${totalCitations - validCitations}`);
    console.log(`  Validation rate: ${validPercentage.toFixed(1)}%`);

    // Check peer-reviewed requirement
    const peerReviewedCount = countPeerReviewed(bibEntries);
    const peerReviewedPercentage = bibEntries.length > 0 ? (peerReviewedCount / bibEntries.length) * 100 : 0;

    console.log(`\nPeer-reviewed sources:`);
    console.log(`  Peer-reviewed entries: ${peerReviewedCount}`);
    console.log(`  Total entries: ${bibEntries.length}`);
    console.log(`  Peer-reviewed percentage: ${peerReviewedPercentage.toFixed(1)}%`);

    if (peerReviewedPercentage < 50) {
        console.log(`  ❌ FAIL: Peer-reviewed requirement not met (need 50%, got ${peerReviewedPercentage.toFixed(1)}%)`);
    } else {
        console.log(`  ✓ PASS: Peer-reviewed requirement met (${peerReviewedPercentage.toFixed(1)}%)`);
    }

    // Overall result
    if (validPercentage === 100 && peerReviewedPercentage >= 50) {
        console.log('\n✓ All citations validated successfully!');
        process.exit(0);
    } else {
        console.log('\n✗ Citation validation failed!');
        process.exit(1);
    }
}

// Parse bibliography file to extract entries
function parseBibFile(content) {
    const entries = [];
    const lines = content.split('\n');
    let currentEntry = null;
    let currentType = null;
    let currentKey = null;

    for (const line of lines) {
        // Match entry type and key: @type{key,
        const entryMatch = line.match(/^@(\w+)\{([^,]+),/);
        if (entryMatch) {
            if (currentEntry) {
                entries.push(currentEntry);
            }
            currentType = entryMatch[1].toLowerCase();
            currentKey = entryMatch[2].trim();
            currentEntry = {
                type: currentType,
                key: currentKey,
                isPeerReviewed: isPeerReviewedType(currentType)
            };
        }

        // Check for title field to determine if it's academic
        if (line.includes('title=') && currentEntry) {
            const title = line.split('=')[1]?.replace(/[{}"]/g, '').trim();
            if (title) {
                currentEntry.title = title;
            }
        }
    }

    if (currentEntry) {
        entries.push(currentEntry);
    }

    return entries;
}

// Determine if entry type is peer-reviewed
function isPeerReviewedType(type) {
    return ['article', 'inproceedings', 'incollection', 'techreport', 'phdthesis', 'mastersthesis', 'book'].includes(type.toLowerCase());
}

// Find citations in markdown content
function findCitations(content) {
    // Look for APA-style citations: (Author, Year) or Author (Year)
    const citationRegex = /\(?[A-Z][a-z]+,\s*\d{4}\)?/g;
    const matches = content.match(citationRegex) || [];

    // Also look for more complex citations
    const complexCitationRegex = /\(?[A-Z][a-z]+,\s*\d{4}[a-z]?\)?/g;
    const complexMatches = content.match(complexCitationRegex) || [];

    // Combine and deduplicate
    const allMatches = [...new Set([...matches, ...complexMatches])];

    return allMatches;
}

// Validate if a citation exists in bibliography
function validateCitation(citation, bibEntries) {
    // Extract year from citation
    const yearMatch = citation.match(/\d{4}/);
    if (!yearMatch) return false;
    const year = yearMatch[0];

    // Extract author from citation
    const authorMatch = citation.match(/[A-Z][a-z]+/);
    if (!authorMatch) return false;
    const author = authorMatch[0];

    // Look for matching entry in bibliography
    return bibEntries.some(entry => {
        // Check if entry key contains author and year, or if title contains author
        return entry.key.toLowerCase().includes(author.toLowerCase()) && entry.key.includes(year) ||
               entry.title && entry.title.toLowerCase().includes(author.toLowerCase());
    });
}

// Count peer-reviewed entries
function countPeerReviewed(bibEntries) {
    return bibEntries.filter(entry => entry.isPeerReviewed).length;
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
validateCitations();