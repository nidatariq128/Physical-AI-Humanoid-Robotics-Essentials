---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: NG TECH
command: /sp.constitution
labels: ["constitution","governance","specification"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

You are writing the project constitution for an AI-assisted, spec-driven book creation project built using Spec-Kit Plus, Docusaurus, and deployed to GitHub Pages.

The constitution you produce will act as the authoritative governance document for all future specifications, chapters, research, citations, and generated content in this project.
It must be explicit, enforceable, and written in a formal academic-technical tone.

Project Overview

The project is Project 1, an AI-driven book authored with the assistance of Claude Code under a strict specification-first workflow.
The book is intended for an academic audience with a computer science background and must meet peer-review–level rigor.

Core Principles

Define and enforce the following principles as non-negotiable rules:

Accuracy Through Primary Source Verification

Every factual, technical, historical, or statistical claim must be verified against primary or authoritative secondary sources.

Unsupported claims are prohibited.

Clarity for an Academic Computer Science Audience

Writing must assume familiarity with foundational computer science concepts.

Terminology must be precise, consistent, and formally defined where required.

Reproducibility

All claims, arguments, and conclusions must be traceable to verifiable sources.

Readers must be able to independently validate claims using provided citations.

Scholarly Rigor

Peer-reviewed literature is strongly preferred.

Informal sources may only be used when no peer-reviewed alternative exists and must be clearly justified.

Key Standards

Explicitly codify the following standards:

Citation Requirements

All factual claims must include citations.

Citation style must be APA.

Inline citations and a complete references section are mandatory.

Source Quality

A minimum of 50% of all sources must be peer-reviewed journal articles or conference papers.

Remaining sources may include academic books, standards documents, or authoritative technical documentation.

Plagiarism Policy

Zero tolerance for plagiarism.

All generated text must be original and properly paraphrased.

Direct quotations must be clearly marked and cited.

Writing Clarity and Readability

Target Flesch-Kincaid Grade Level: 10–12.

Sentences must be concise, formal, and logically structured.

Constraints

The constitution must enforce the following hard constraints for the final book:

Total Word Count: 5,000–7,000 words

Minimum Number of Sources: 15

Final Output Format: PDF

Citations: Embedded, properly formatted APA citations throughout the text

Success Criteria

Define success criteria that must be satisfied before the book is considered complete:

All claims are verified and traceable to cited sources

Zero plagiarism detected by automated and manual review

Passes independent fact-checking review

Meets all formatting, citation, and readability requirements

Enforcement and Governance

The constitution must:

Require rejection or revision of any content that violates its rules

Mandate continuous self-auditing for citation completeness, factual accuracy, and plagiarism

Output Requirements

Produce a single, complete constitution document

Use clear section headings

Write in formal academic-technical English

Do not include meta commentary or explanations outside the constitution itself

## Response snapshot

Created the project constitution for the AI-assisted, spec-driven book creation project. The constitution includes six core principles: Accuracy Through Primary Source Verification, Clarity for an Academic Computer Science Audience, Reproducibility, Scholarly Rigor, Zero-Tolerance Plagiarism Policy, and Citation Completeness. It also defines standards and requirements, development workflow, and governance procedures. The constitution enforces all the requirements specified by the user, including citation requirements (APA format), source quality standards (50% peer-reviewed), plagiarism policy, writing standards (grade level 10-12), content constraints (5,000-7,000 words, minimum 15 sources), and quality assurance procedures.

## Outcome

- ✅ Impact: Created project constitution document that establishes governance framework for AI-assisted book creation project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with complete constitution
- 🔁 Next prompts: Continue with spec creation, planning, or implementation tasks as defined by the new constitution
- 🧠 Reflection: The constitution successfully incorporates all required principles and standards, establishing a strong governance foundation for the project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - all requirements met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating project specifications based on the established constitution