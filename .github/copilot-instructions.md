# Integrated RAG Chatbot — Copilot Instructions (Short)

Scope: Embed a RAG chatbot in a published book using OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier. It must answer questions about the book’s content and strictly support “selection-only” answers when the user highlights text. Assume the book content is prepared and ready for ingestion.

## 0) SpecKit+ Workflow (No vibe coding)
- Only use Spec Kit slash commands; do not handcraft ad‑hoc steps.
  1. /speckit.constitution
  2. /speckit.specify
  3. /speckit.plan
  4. /speckit.tasks
  5. /speckit.implement
- Every run must update history and prompt files (see Persistence) and commit them.

## 1) What to Build
- Book-wide Q&A with grounded citations (chapter/section/page/URI).
- Selection-only mode: answer strictly from highlighted text (and only immediate neighbors if explicitly enabled); otherwise return “insufficient evidence.”
- Backend: FastAPI endpoints for chat, retrieval, selection.
- Persistence: Neon tables for sessions, messages, selections, citations.
- Retrieval: Qdrant over chunked book content with OpenAI embeddings.
- Orchestration: OpenAI Agents/ChatKit for prompts and streaming answers.

## 2) Minimal Plan
- Ingest: Parse → chunk by structure/length → embed (OpenAI) → upsert to Qdrant (metadata: chapter/section/page/URI).
- Retrieve: Vector search; apply selection-only filters when selection exists; rank by similarity and proximity.
- Generate: ChatKit prompts with retrieved chunks; stream response; always include citations.
- Persist: Save sessions, messages, selections, citations in Neon Postgres.
- UI: Reader integrates selection capture and a chat sidebar with a selection-only toggle.

## 3) Endpoints (FastAPI)
- POST /chat: {question, selection?{text, chapter, section, page}, session_id} → streamed answer + citations; enforce selection-only when provided.
- POST /retrieve: internal retrieval with filters.
- POST /selection: store selection metadata.
- GET /history?session_id=: return messages + citations.

## 4) Tasks (Condensed)
- Implement scripts/ingest_book.py to chunk/embed/upsert to Qdrant.
- Backend: main.py, routes/chat.py, routes/retrieve.py, routes/selection.py.
- Storage: storage/qdrant.py, storage/postgres.py; Neon schema for sessions/messages/selections/citations.
- Frontend: selection capture + chat sidebar; show streamed answers and citations.
- Tests: selection-only enforcement; citation presence; basic e2e Q&A.

## 5) Prompts (Repo Files)
- .specify/memory/prompts/system.md: book-only grounding, mandatory citations, strict selection-only policy.
- .specify/memory/prompts/assistant.md: concise, scholarly; refusal on weak evidence; list sources.
- .specify/memory/prompts/user_templates.md: templates for normal and selection-only Q&A.

## 6) History & Prompt Persistence (Explicit)
- Create and continuously update:
  - .specify/memory/history/{user}-{YYYYMMDD}.json (all sessions/messages/selections/citations refs)
  - .specify/memory/prompts/system.md, assistant.md, user_templates.md (versioned as they evolve)
- All changes must be produced via SpecKit+ commands and captured in commits/PRs. No manual “vibe coding.”

## 7) Risk Management
- Strict grounding; refuse outside-book answers.
- Validate selections; return “insufficient evidence” when needed.
- Graceful degradation on Qdrant/Neon outages; minimal data retention for privacy.
- Keep latency reasonable via chunk sizing and top-k tuning.