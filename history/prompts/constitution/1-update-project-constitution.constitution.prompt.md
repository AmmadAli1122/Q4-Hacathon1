---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude User
command: /sp.constitution
labels: [constitution, setup, governance]
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

```
/sp.constitution

Project: AI-Spec–Driven Unified Book with Embedded RAG Chatbot

Objective:
Design, author, deploy, and augment a technical book using AI-native development workflows.
The book must be written using Spec-Kit Plus and Claude Code, published via Docusaurus,
deployed to GitHub Pages, and enhanced with an embedded Retrieval-Augmented Generation (RAG)
chatbot capable of answering queries grounded strictly in the book's content.

────────────────────────────────────────
CORE PRINCIPLES
────────────────────────────────────────

1. Specification-First Development
   - All content, architecture, and implementation decisions must originate from clear,
     version-controlled specifications.
   - Specs must be executable, testable, and auditable.

2. Content Groundedness & Verifiability
   - All factual, technical, and conceptual claims must be traceable to either:
     a) the book's authored content, or
     b) explicitly cited external sources.
   - The RAG chatbot must never hallucinate beyond indexed material.

3. AI-Native Authoring Discipline
   - Claude Code is used as a collaborative coding and writing agent, not an autonomous authority.
   - Human-readable structure and AI-readable specs must coexist without conflict.

4. Transparency & Reproducibility
   - Any build, deployment, or indexing step must be reproducible from repository state.
   - Configuration files, prompts, and schemas must be committed and documented.

5. User-Centric Knowledge Access
   - The embedded chatbot must prioritize accuracy, explainability, and contextual relevance,
     especially when answering questions from user-selected text.

────────────────────────────────────────
CONTENT STANDARDS (BOOK)
────────────────────────────────────────

- Writing Style:
  - Technical, structured, and instructional
  - Target audience: software engineers, AI practitioners, advanced students
  - Clarity level: Flesch–Kincaid grade 10–12

- Structure:
  - Modular chapters compatible with Docusaurus
  - Clear navigation, headings, and cross-references
  - Code examples must be runnable and explained

- Citations:
  - Citation style: APA
  - All external references must be listed and linked
  - Minimum 40% authoritative sources (official docs, standards, peer-reviewed papers)

- Integrity:
  - Plagiarism tolerance: 0%
  - AI-generated text must be reviewed for factual accuracy and originality

────────────────────────────────────────
TECHNICAL STANDARDS (RAG CHATBOT)
────────────────────────────────────────

Architecture Requirements:
- Backend: FastAPI
- LLM Integration: OpenAI Agents / ChatKit SDKs
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Storage: Neon Serverless Postgres
- Deployment-ready configuration

RAG Constraints:
- Retrieval must be limited to indexed book content
- Support two modes:
  1. Global book-wide question answering
  2. Context-restricted answering based only on user-selected text
- Explicitly cite retrieved passages in responses
- No response allowed if relevant context is absent

Data Handling:
- Chunking strategy must preserve semantic coherence
- Embeddings must be versioned and regenerable
- Index updates must sync with book content changes

────────────────────────────────────────
DEPLOYMENT & INTEGRATION STANDARDS
────────────────────────────────────────

- Book:
  - Built using Docusaurus
  - Hosted on GitHub Pages
  - CI/CD pipeline documented

- Chatbot:
  - Embedding in Docusaurus pages with clear UI for user interaction.
```

## Response snapshot

I have successfully updated the project constitution at `.specify/memory/constitution.md` with the specific requirements for the "AI-Spec–Driven Unified Book with Embedded RAG Chatbot" project.

### Changes Made:
- **Version**: Updated to 1.0.0 (initial version for this project)
- **Project Name**: "AI-Spec–Driven Unified Book with Embedded RAG Chatbot"
- **Core Principles**: 6 principles added based on your requirements:
  1. Specification-First Development
  2. Content Groundedness & Verifiability
  3. AI-Native Authoring Discipline
  4. Transparency & Reproducibility
  5. User-Centric Knowledge Access
  6. Quality and Integrity Standards

- **Technical Standards Section**: Added RAG Chatbot architecture requirements
- **Deployment Standards Section**: Added Docusaurus and GitHub Pages deployment requirements
- **Governance Section**: Added project governance rules

### Validation Completed:
- All placeholder tokens have been replaced with concrete values
- No remaining bracket tokens in the file
- Version and dates are in correct ISO format (2025-12-25)
- File structure and formatting preserved correctly

### Files Updated:
- `.specify/memory/constitution.md` - Complete project constitution with all specific requirements

### Suggested Commit Message:
`docs: add constitution for AI-Spec-Driven Book with RAG Chatbot project v1.0.0`

The constitution now fully reflects the project requirements you provided, including all technical standards for the RAG chatbot, content standards, and deployment requirements.

## Outcome

- ✅ Impact: Constitution document created with all project-specific principles and standards
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Implementation can now proceed with updated governance structure
- 🧠 Reflection: Project now has clear governance framework for development

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A