# Project Constitution — Physical AI & Humanoid Robotics Textbook (Spec-Driven)

## Project Name
"Physical AI & Humanoid Robotics" AI-Native Textbook + RAG Chatbot

## Mission
Build an AI-native textbook (Docusaurus) that teaches Physical AI & Humanoid Robotics and ships with an embedded Retrieval-Augmented Generation (RAG) chatbot that can answer user questions about the book content — including answering based only on text the user selects. Provide optional user signup, personalization, and Urdu translation toggles to maximize accessibility and competition bonus points.

## Vision Principles
- AI-first: The book content and tools are designed to be consumed interactively by AI agents (Claude Code subagents, OpenAI Agents/ChatKit).
- Reusable Intelligence: Encapsulate reusable subagents/skills for content generation, summarization, and personalization.
- Accessibility & Localisation: Provide an easy Urdu translation toggle and personalized content based on user background.
- Reproducible and Spec-Driven: Use Spec-Kit Plus for spec-driven development; all features described as verifiable specs and tasks.
- Open & Documented: All code, specs, and prompts are versioned and placed in the repository for reproducibility.

## Scope (In-Scope)
- Docusaurus-based textbook with full content structure for the course.
- Embedded RAG chatbot: FastAPI backend, Neon Serverless Postgres for user data (signup), Qdrant Cloud for vector store, RAG retrieval, and OpenAI Agents/ChatKit (or equivalent) for agent orchestration.
- Ability for chatbot to answer from a user-selected text snippet (exact-match RAG scope).
- Signup/Signin with Better-Auth integration (bonus).
- Per-user personalization and per-chapter personalization button (bonus).
- Urdu translation toggle per chapter (bonus).
- CI/CD to deploy site to GitHub Pages; tests for RAG pipeline components.

## Non-Goals (Out-of-Scope)
- Building a physical humanoid robot (hardware manufacturing).
- Long-term hosting cost guarantees; demo-scale deployment only.
- Proprietary closed-source agent features (we will use OpenAPI/OSS integrations unless explicitly licensed).

## Constraints & Assumptions
- Primary repo will be hosted on GitHub; user will supply owner/repo for automated PR generation.
- High computational workloads (Isaac Sim) are out of scope for the CI environment; simulation assets are delivered as references and examples; heavy compute runs locally or in cloud workstations.
- Qdrant will be used as vector store (Qdrant Cloud Free Tier). Neon Serverless Postgres used for user records.
- Models: The project will support both Claude Code subagents and OpenAI/ChatKit Agents; exact provider selection is configurable.

## Privacy & Security
- User credentials/PII must never be stored in vectors. Only sanitized metadata and snippets are stored.
- Use Neon Postgres to store user profiles and consent flags. Use explicit opt-in for storing user-generated data for personalization and analytics.
- Secrets (API keys) must be stored in GitHub Secrets and never committed.

## Success Criteria
- Book deployed to GitHub Pages or Vercel and accessible publicly.
- RAG chatbot embedded in book, can answer book-content questions with >= 90% accuracy on a predefined acceptance test suite (see spec.md acceptance tests).
- Chatbot respects "answer from selected text only" mode: when user selects a passage and requests answers constrained to selection, responses strictly reference only selection (automated checker).
- Signup/Signin via Better-Auth integrated and storing profile information (bonus).
- Personalization per chapter works for logged-in users (bonus).
- Urdu translation toggle functions and produces readable translations (bonus).

## Governance
- Use the spec-driven workflow: all significant changes originate as edits to spec.md and tasks.md and result in PRs referencing the spec ID.
- Every PR must reference the associated task ID(s), include a test plan, and follow PR checklist.

## Licensing & Attribution
- Code: MIT (default). Content: CC-BY-SA 4.0 (unless otherwise specified).
- Attribution sections required in the book for spec-kit, Claude Code, and any third-party content.

## Revision Policy
- This constitution is the canonical source of project constraints. Update by PR and majority approval from maintainers.