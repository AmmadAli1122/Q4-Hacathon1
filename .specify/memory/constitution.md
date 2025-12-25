<!-- SYNC IMPACT REPORT
Version change: N/A (initial constitution) → 1.0.0
List of modified principles: N/A (initial creation)
Added sections: All principles and sections (initial constitution)
Removed sections: None
Templates requiring updates: N/A
Follow-up TODOs: None
-->

# AI-Spec–Driven Unified Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Specification-First Development
All content, architecture, and implementation decisions must originate from clear, version-controlled specifications. Specs must be executable, testable, and auditable.

### II. Content Groundedness & Verifiability
All factual, technical, and conceptual claims must be traceable to either: a) the book's authored content, or b) explicitly cited external sources. The RAG chatbot must never hallucinate beyond indexed material.

### III. AI-Native Authoring Discipline
Claude Code is used as a collaborative coding and writing agent, not an autonomous authority. Human-readable structure and AI-readable specs must coexist without conflict.

### IV. Transparency & Reproducibility
Any build, deployment, or indexing step must be reproducible from repository state. Configuration files, prompts, and schemas must be committed and documented.

### V. User-Centric Knowledge Access
The embedded chatbot must prioritize accuracy, explainability, and contextual relevance, especially when answering questions from user-selected text.

### VI. Quality and Integrity Standards
All content must meet technical writing standards with Flesch–Kincaid grade 10–12 readability, proper citations (APA style), and 0% plagiarism tolerance. Code examples must be runnable and explained.

## Technical Standards (RAG Chatbot)
Architecture Requirements: Backend: FastAPI, LLM Integration: OpenAI Agents / ChatKit SDKs, Vector Database: Qdrant Cloud (Free Tier), Relational Storage: Neon Serverless Postgres, Deployment-ready configuration. RAG Constraints: Retrieval must be limited to indexed book content, Support two modes: 1. Global book-wide question answering, 2. Context-restricted answering based only on user-selected text, Explicitly cite retrieved passages in responses, No response allowed if relevant context is absent. Data Handling: Chunking strategy must preserve semantic coherence, Embeddings must be versioned and regenerable, Index updates must sync with book content changes.

## Deployment & Integration Standards
Book: Built using Docusaurus, Hosted on GitHub Pages, CI/CD pipeline documented. Chatbot: Embedding in Docusaurus pages with clear UI for user interaction.

## Governance
This constitution represents the highest authority for development practices. All team members are required to follow these principles. Deviations must be documented as technical debt and addressed in subsequent work. Amendments require team consensus and documented approval.

**Version**: 1.0.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25