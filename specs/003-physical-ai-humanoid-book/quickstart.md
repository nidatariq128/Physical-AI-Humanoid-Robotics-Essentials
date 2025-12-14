# Quickstart Guide: AI Robotics Book Development

## Prerequisites

- Git installed and configured
- Node.js 18+ (for Docusaurus)
- Python 3.8+ (for automation scripts)
- Pandoc (for PDF generation)
- Access to academic databases for research

## Setup Instructions

### 1. Clone and Initialize Repository

```bash
git clone <repository-url>
cd ai-robotics-book
npm install
```

### 2. Environment Configuration

```bash
# Install Python dependencies
pip install -r requirements.txt

# Setup citation verification tools
npm install -g citation-checker
```

### 3. Project Structure Overview

The project is organized as follows:

```
├── docs/                   # Content source files
│   ├── foundations/        # AI Robotics foundations
│   ├── ros2/              # ROS 2 module content
│   ├── simulation/        # Simulation environments
│   ├── perception/        # AI perception content
│   ├── vla/               # Vision-Language-Action systems
│   ├── capstone/          # Capstone project content
│   └── hardware/          # Hardware architectures
├── src/                   # Build and automation scripts
├── _config/               # Configuration files
└── specs/                 # Feature specifications and plans
```

## Writing Workflow

### 1. Research Phase
- Identify authoritative sources for each section
- Verify technical claims against documentation
- Ensure at least 50% peer-reviewed sources

### 2. Content Creation
- Create new markdown files in appropriate module directory
- Include APA citations for all factual claims
- Maintain Flesch-Kincaid Grade Level 10-12

### 3. Validation Process
- Run citation verification: `npm run check-citations`
- Verify readability: `npm run readability-check`
- Ensure word count compliance: `npm run word-count`

## Key Commands

### Content Development
```bash
# Start local development server
npm run start

# Build static site
npm run build

# Generate PDF from content
npm run pdf

# Check all citations
npm run validate-citations

# Run plagiarism check
npm run plagiarism-check
```

### Quality Assurance
```bash
# Run all validation checks
npm run validate-all

# Check for constitutional compliance
npm run constitution-check

# Verify word count requirements
npm run word-count-report

# Run readability analysis
npm run readability-report
```

## Module Development

### Creating a New Section
1. Determine the appropriate module directory
2. Create a new markdown file with descriptive name
3. Include frontmatter with metadata:

```markdown
---
title: "Section Title"
module: "module-name"
word_count: 0
learning_objectives:
  - Objective 1
  - Objective 2
prerequisites:
  - Prerequisite 1
validation_status: draft
---
```

### Adding Citations
- Use proper APA format in text: (Author, Year)
- Add full citations to bibliography.bib file
- Verify citations with `npm run check-citations`

## Quality Standards

### Content Requirements
- Minimum 50% peer-reviewed sources
- All claims must have citations
- Maintain 5,000-7,000 total word count
- Flesch-Kincaid Grade Level 10-12
- Zero plagiarism tolerance

### Review Process
1. Self-review for technical accuracy
2. Citation completeness verification
3. Readability assessment
4. Constitutional compliance check
5. Peer review by domain expert

## Common Tasks

### Adding a New Module
1. Create new directory in `docs/`
2. Add module configuration in `docusaurus.config.js`
3. Create initial sections following template

### Updating Bibliography
1. Add new entries to `_config/bibliography.bib`
2. Verify APA format compliance
3. Run citation verification

### Building Final PDF
```bash
# Generate complete PDF with embedded citations
npm run final-pdf
```

## Troubleshooting

### Citation Issues
- Verify BibTeX format in bibliography file
- Ensure in-text citations match bibliography entries
- Run `npm run check-citations` for detailed report

### Build Problems
- Clear cache: `npm run clear`
- Reinstall dependencies: `npm install`
- Check Docusaurus documentation for theme issues

### Validation Failures
- Check word count compliance
- Verify citation format
- Ensure all constitutional requirements met