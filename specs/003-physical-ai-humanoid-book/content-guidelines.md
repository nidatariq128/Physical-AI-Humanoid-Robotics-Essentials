# Content Creation Guidelines: AI Robotics Book

## Overview

This document provides guidelines for creating content that meets the constitutional and specification requirements for the AI Robotics Book project.

## Constitutional Requirements

### 1. Accuracy Through Primary Source Verification
- Every factual, technical, historical, or statistical claim must be verified against primary or authoritative secondary sources
- Unsupported claims are prohibited
- All content must be traceable to verifiable sources

### 2. Clarity for Academic CS Audience
- Writing must assume familiarity with foundational computer science concepts
- Terminology must be precise, consistent, and formally defined where required
- Content must be accessible to readers with computer science backgrounds at the academic level
- Maintain formal academic-technical tone throughout

### 3. Reproducibility
- All claims, arguments, and conclusions must be traceable to verifiable sources
- Readers must be able to independently validate claims using provided citations
- Every assertion must be supported by documented evidence

### 4. Scholarly Rigor
- Prioritize peer-reviewed literature for all factual claims
- Informal sources may only be used when no peer-reviewed alternative exists and must be clearly justified
- Content must meet peer-review-level rigor and academic standards

### 5. Zero-Tolerance Plagiarism Policy
- Complete prohibition of plagiarism in all forms
- All generated text must be original and properly paraphrased
- Direct quotations must be clearly marked and cited using proper APA format

### 6. Citation Completeness
- All factual claims must include proper citations following APA format
- Inline citations and a complete references section are mandatory
- At least 50% of all sources must be peer-reviewed journal articles or conference papers

## Writing Standards

### Flesch-Kincaid Grade Level
- Target: 10-12 grade level
- Use tools to verify readability before submission
- Complex concepts should be explained clearly without oversimplification

### Word Count
- Total book: 5,000-7,000 words
- Individual sections should be appropriately sized (typically 300-1000 words per section)
- Balance depth with brevity

## Content Structure

### Frontmatter Template
Each content file should begin with proper frontmatter:

```markdown
---
title: "Section Title"
module: "module-name"  # foundations, ros2, simulation, perception, vla, capstone, hardware
word_count: 0  # Update this after writing
learning_objectives:
  - "Objective 1"
  - "Objective 2"
prerequisites:
  - "Prerequisite 1"
validation_status: draft  # draft, reviewed, validated, published
---
```

### Section Structure
1. **Title** - Clear, descriptive title
2. **Learning Objectives** - What readers should learn from this section
3. **Prerequisites** - What knowledge is needed before reading
4. **Main Content** - The body with proper citations
5. **Summary** - Key points recap

## Citation Guidelines

### APA Format
Use proper APA format for in-text citations: (Author, Year) or Author (Year, p. X)

Examples:
- Single author: (Smith, 2023) or Smith (2023) showed...
- Multiple authors: (Johnson et al., 2022)
- Page-specific: (Brown, 2021, p. 145)

### Bibliography Management
- Add all sources to `_config/bibliography.bib`
- Use BibTeX format
- Ensure at least 50% are peer-reviewed sources

## Quality Assurance

### Validation Steps
Before marking content as complete, run:
```bash
npm run validate-all
```

This will check:
- Word count requirements
- Readability (Grade 10-12)
- Citation completeness
- Plagiarism detection
- Constitutional compliance

### Review Process
1. Self-review for technical accuracy
2. Citation completeness verification
3. Readability assessment
4. Constitutional compliance check
5. Peer review by domain expert

## Module-Specific Guidelines

### Foundations Module
- Focus on conceptual understanding
- Bridge digital AI to physical robotics
- Explain embodied intelligence principles

### ROS 2 Module
- Practical examples with code snippets
- Step-by-step implementation guides
- Best practices for robotic communication

### Simulation Module
- Compare Gazebo and Unity approaches
- Include URDF/SDF examples
- Cover physics simulation concepts

### Perception Module
- Emphasize NVIDIA Isaac tools
- Include practical SLAM examples
- Cover navigation and path planning

### VLA Module
- Focus on multi-modal integration
- Include speech recognition examples
- Cover LLM integration for planning

### Capstone Module
- Integrate concepts from all previous modules
- Provide end-to-end workflow
- Include sim-to-real considerations

### Hardware Module
- Document multiple tier options
- Include cost considerations
- Cover deployment constraints

## File Organization

- Content files in appropriate module directories: `docs/module-name/`
- Images and assets in `docs/assets/`
- Code examples in `src/examples/` if needed
- Maintain consistent naming conventions

## Submission Process

1. Create content using the template
2. Add proper citations
3. Verify word count and readability
4. Run validation scripts
5. Update frontmatter with correct word count
6. Submit for review