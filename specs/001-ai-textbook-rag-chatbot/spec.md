# Spec — "Physical AI & Humanoid Robotics" AI-Native Textbook + RAG Chatbot

## Overview (Abstract)
Create a Docusaurus-hosted textbook for a course "Physical AI & Humanoid Robotics" and embed a RAG chatbot that can answer questions about the textbook content. The chatbot must support:
- Free Q&A over full-book corpus.
- Constrained Q&A using only user-selected text (strict selection mode).
- Session-level personalization based on user profile (collected at signup).
- Optional Urdu translation toggle on chapters and the ability to personalize translations.

This spec is intended to be used with Spec-Kit Plus / Claude Code to generate branches and PRs that implement the features.

## Problem Statement (for Spec-Kit Plus / github-coding-agent)
Replace repository placeholder with actual owner/repo before executing.

problem_title: "Implement AI-Native Textbook + Embedded RAG Chatbot"

problem_statement:
- Build a Docusaurus textbook from course content provided in curriculum markdown.
- Implement a RAG chatbot embedded inside the Docusaurus site using a FastAPI backend and a front-end widget.
- Backend responsibilities:
  - Index textbook content into a vector store (Qdrant cloud).
  - Provide vector retrieval APIs with a 'scope' flag: "fullbook" or "selected_text".
  - Implement a secure user profile store (Neon Serverless Postgres) and Better-Auth signup/signin integration.
  - Provide endpoints for personalization (store user background and preferences) and translation (request Urdu translation via a translation subagent or LLM).
- Frontend responsibilities:
  - Add a chat widget to the Docusaurus site that can:
    - Query backend for answers.
    - When user selects text and clicks "Ask about selection", send the selected text to the backend and request an answer constrained to that selection.
    - Offer a per-chapter "Personalize" button that applies user profile preferences to chapter content (client-side rendering/per-chapter override).
    - Offer an "Urdu" toggle per chapter; when activated, fetch translated content from backend or agent.
- Acceptance criteria (automated tests):
  - Indexing test: given sample doc set, index documents and retrieve top-k relevant results.
  - Selection mode test: given a question and a selected snippet, the LLM response must contain only facts present in snippet (verified via exact-match fact-checker).
  - Signup test: Better-Auth flow creates a user record in Neon; profile saved.
  - Personalization test: per-chapter content rendering changes when a user profile is applied.
  - Translation test: Urdu translations exist for sample chapter (readability check using BLEU or human spot check).
- Deliverables:
  - Docusaurus site with textbook content.
  - FastAPI backend with the required endpoints and tests.
  - Vectorization pipeline (script to ingest docs to Qdrant).
  - Deployment configs (GitHub Actions) to deploy book to GitHub Pages.
  - README describing how to run locally and how to deploy.
  - Spec files and task references for all major features.

repository: "owner/repo"  # <- replace with your repository

## Architecture (high-level)
- Frontend: Docusaurus + React chat widget
- Backend: FastAPI (Python) with endpoints:
  - POST /api/index -> index documents to Qdrant
  - POST /api/retrieve -> query Qdrant (params: query_text, scope: fullbook|selected_text, user_id)
  - POST /api/answer -> call LLM/Agent with retrieved docs and question (system prompt enforces selection constraints)
  - POST /api/signup -> Better-Auth redirect/flow + create record in Neon
  - GET /api/user/{id}/profile -> return personalization preferences
  - POST /api/translate -> request Urdu translation for chapter id or selection
- Vector DB: Qdrant (cloud)
- Metadata DB: Neon Serverless Postgres (user profiles, mappings)
- Agents: Claude Code subagents for content generation and translation; OpenAI/ChatKit Agents for the RAG answering pipeline (configurable)
- Hosting: GitHub Pages (book), Vercel or Netlify optional for RT site; FastAPI hosted on platform of choice (Cloud Run/Heroku/Vercel serverless).

## Data Model (examples)
- document:
  - id (string)
  - chapter_id (string)
  - content (text)
  - embedding (vector)
  - metadata: {title, chapter, section, url, lang}
- user_profile:
  - id
  - name
  - background (hardware/software string)
  - preferences: {difficulty_level, examples_preference, localization: 'en'|'ur'}
  - consent_flags
- answer_session:
  - session_id
  - user_id
  - question
  - scope ('fullbook'|'selected_text')
  - selected_text (if any)
  - retrieved_ids
  - model_response

## Prompts (system-level) — for the answer endpoint
- System prompt (RAG): "You are an assistant answering questions strictly based on the retrieved documents. If scope=='selected_text', you may not use any information outside the provided selected_text. If the question requires knowledge not present in retrievals, respond with 'I don't know — the provided text does not contain enough information.'"
- Personalization layer: Before final answer, a subagent re-writes the output to match user's background (e.g., more/less detail).

## Acceptance Tests & Metrics
- RAG accuracy: run a test set of 50 Q/A pairs from the book. Responses must contain the correct reference and no invented facts in 90% of cases.
- Selection mode: 100% of returned facts must be present in selected text (verified by exact string match or semantic anchors).
- Signup flow: End-to-end Better-Auth signup should create a Neon user with profile attributes.
- UI tests: Chat widget loads in Docusaurus and handles selection mode and personalization toggles.

## Bonus Features (extra-scoring)
- Claude Code Subagents and Agent Skills implemented and exposed as reusable functions.
- Better-Auth signup and profile-driven personalization enabled.
- Per-chapter "Personalize" button that rewrites chapter content using the user's stored background.
- Per-chapter Urdu translation toggle integrated and persisted.

## Ops & Deployment
- GitHub Actions:
  - build/docs: build and preview Docusaurus site
  - ci/backend: run tests for FastAPI and indexing pipeline
  - deploy/pages: push built book to gh-pages branch
- Secrets:
  - QDRANT_API_KEY, NEON_URL, BETTER_AUTH_CLIENT_ID/SECRET, OPENAI_API_KEY / CLAUDE_API_KEY
- Local dev:
  - Dev Docker Compose: FastAPI + qdrant local + Postgres local for dev

## Problem IDs & Tracing
- Each feature and task must include a spec id (e.g., SPEC-001-rag-indexing). Map PRs to spec ids.

## Example commands to run locally
- python scripts/ingest_to_qdrant.py --docs ./docs --collection physical_ai_humanoid_robotics_course
- uvicorn app.main:app --reload

---

# Module Specifications (detailed)
Below are module-level specs for Modules 1–4. Each module maps to SPEC IDs and includes deliverables, acceptance criteria, sample chapter file names and content structure, tests, and suggested exercises.

---

## Module 1 — The Robotic Nervous System (ROS 2)
spec_id: SPEC-M1-ROS2

Summary:
Module 1 teaches ROS 2 fundamentals: nodes, topics, services, actions, rclpy (Python), URDF for humanoids, and bridging LLM-based agents to ROS controllers.

Learning Objectives:
- Explain ROS 2 architecture and communication primitives.
- Build ROS 2 packages using Python (rclpy) that publish/subscribe to topics, provide services, and use actions.
- Create a simple URDF model for a robot limb and understand joint/state publishing.
- Demonstrate how an LLM or agent can send high-level commands that are translated into ROS 2 action sequences.

Scope:
- Intro lessons on nodes/topics/services/actions.
- rclpy examples: publisher/subscriber, service server/client, action server/client.
- URDF basics using xacro and simple humanoid limb example.
- Agent integration pattern (LLM -> planner -> ROS 2 commands).

Deliverables:
- Docs: docs/module-01-ros2/introduction.md, nodes-topics-services.md, urdf-basics.md, agent-bridge.md
- Code samples in /examples/module1/:
  - ros2_pkg_publisher_subscriber/
  - ros2_pkg_service_action/
  - urdf_example/
  - agent_bridge_stub/ (example demonstrating LLM -> action client)
- Ingestion: markdown converted to docs/ and indexed into Qdrant.

Acceptance Criteria & Tests:
- Unit tests for example nodes (mocked rclpy interfaces) exist and run in CI.
- Indexing test: docs for Module 1 produce embeddings and top-k retrieval returns correct sections for queries like "how to create a service in rclpy".
- Agent-bridge test: given a high-level command "pick up the block", the planner service returns a verified sequence of ROS 2 actions (unit test with mocked action server).

Sample Exercises (for content):
- Lab 1.1: Implement a publisher node that publishes sensor-like data (topic: /sim/sensor) and a subscriber that logs data.
- Lab 1.2: Implement a service to set robot mode and write a client that calls it.
- Lab 1.3: Create a URDF of a 2-joint arm and visualize in rviz (instructions + expected URDF file).

Suggested Chapter Filenames:
- docs/module-01-ros2/01-introduction.md
- docs/module-01-ros2/02-nodes-topics-services.md
- docs/module-01-ros2/03-actions-and-services.md
- docs/module-01-ros2/04-urdf-and-robot-description.md
- docs/module-01-ros2/05-agent-bridge.md
- docs/module-01-ros2/exercises.md

Spec->Tasks mapping:
- TASK-M1-010: Add Module 1 docs and example code (SPEC-M1-ROS2)
- TASK-M1-020: Index Module 1 docs to Qdrant (SPEC-M1-ROS2)
- TASK-M1-030: Implement agent-bridge stub and tests (SPEC-M1-ROS2)

Indexing & Embedding Guidance:
- Chunking: split by headings (H2/H3) into ~400-800 token chunks.
- Metadata: chapter_id = "module-01-ros2", section, file_url, lang=en.
- Anchors: include code snippet labels and exercise ids to support retrieval.

---

## Module 2 — The Digital Twin (Gazebo & Unity)
spec_id: SPEC-M2-DT

Summary:
Module 2 covers physics simulation, URDF/SDF, environment building in Gazebo and Unity, sensor simulation (LiDAR, depth cameras, IMU), and interfacing simulation with ROS 2.

Learning Objectives:
- Set up Gazebo and spawn URDF/SDF robots.
- Create simulation worlds and populate with obstacles and sensors.
- Configure and simulate sensors: RealSense-like depth streams, IMU, LiDAR.
- Interface simulation with ROS 2 topics, and perform playback & bagging.

Scope:
- Gazebo setup, URDF to SDF conversion, Gazebo plugins.
- Unity basics for visualization (optional; overview & workflow).
- Sensor simulation and calibration.
- Simulation best practices for sim-to-real transfer.

Deliverables:
- Docs: docs/module-02-digital-twin/ (setup, sensors, urdf-sdf, plugin examples)
- Examples: /examples/module2/gazebo_worlds/, /examples/module2/sensor_plugins/
- Scripts to launch sample world and ROS 2 bridges.

Acceptance Criteria & Tests:
- CI tests check that launch files parse and that URDF is syntactically valid (linting).
- Retrieval test: queries like "how to simulate a RealSense in Gazebo" return the correct doc chunk.
- Simulation smoke test instructions: documented steps for running a headless Gazebo world (not run in CI due to heavy compute) with expected topics visible.

Sample Exercises:
- Lab 2.1: Create a Gazebo world with a flat plane, spawn a 2-joint robot, and publish IMU data to /imu.
- Lab 2.2: Configure a depth camera plugin and visualize depth stream using rqt_image_view.

Suggested Chapter Filenames:
- docs/module-02-digital-twin/01-gazebo-setup.md
- docs/module-02-digital-twin/02-urdf-to-sdf.md
- docs/module-02-digital-twin/03-sensor-simulation.md
- docs/module-02-digital-twin/04-unity-visualization.md
- docs/module-02-digital-twin/exercises.md

Spec->Tasks mapping:
- TASK-M2-010: Add Module 2 docs & sample worlds (SPEC-M2-DT)
- TASK-M2-020: Add doc linting and URDF validation script (SPEC-M2-DT)
- TASK-M2-030: Index Module 2 docs to Qdrant (SPEC-M2-DT)

Indexing & Embedding Guidance:
- Tag sensor docs with metadata: sensor_type: lidar|depth|imu
- Provide example payloads and topic names in metadata for retrieval.

---

## Module 3 — The AI-Robot Brain (NVIDIA Isaac)
spec_id: SPEC-M3-ISAAC

Summary:
Module 3 focuses on NVIDIA Isaac Sim/Isaac ROS, synthetic data generation, perception (VSLAM), and Nav2 for path planning. This module connects perception and control with hardware acceleration.

Learning Objectives:
- Understand Isaac Sim fundamentals and USD/Omniverse pipeline.
- Create synthetic datasets and train perception models.
- Integrate Isaac ROS components with ROS 2 topics and nodes.
- Build a Nav2-based pipeline for path planning and obstacle avoidance.

Scope:
- Isaac Sim intro and examples, generating synthetic images + labels.
- Integrating trained perception models into Isaac ROS nodes.
- Nav2 configuration for humanoid-like navigation (conceptual & sample configs).

Deliverables:
- Docs: docs/module-03-isaac/ (setup, sim-to-real, synthetic-data, perception)
- Examples: /examples/module3/isaac_sim_configs/, /examples/module3/nav2_configs/
- Scripts: synthetic data generation scripts, example training pipeline stub.

Acceptance Criteria & Tests:
- Documentation for dataset generation and expected dataset manifest.
- Retrieval tests: queries like "how to export USD from Isaac Sim" return correct docs.
- Model integration test: a small stub that demonstrates model inference integration with Isaac ROS (unit testable components, heavy training excluded from CI).

Sample Exercises:
- Lab 3.1: Create a synthetic dataset of 100 images with bounding boxes for a simple object.
- Lab 3.2: Configure a Nav2 YAML file and run a planner in simulation (instructions and expected outcomes).

Suggested Chapter Filenames:
- docs/module-03-isaac/01-intro-isaac.md
- docs/module-03-isaac/02-synthetic-data.md
- docs/module-03-isaac/03-isaac-ros-integration.md
- docs/module-03-isaac/04-nav2-for-humanoids.md
- docs/module-03-isaac/exercises.md

Spec->Tasks mapping:
- TASK-M3-010: Add Module 3 docs and example scripts (SPEC-M3-ISAAC)
- TASK-M3-020: Index Module 3 docs (SPEC-M3-ISAAC)
- TASK-M3-030: Provide synthetic data manifest and example (SPEC-M3-ISAAC)

Indexing & Embedding Guidance:
- Mark simulation asset references with metadata: asset_type, expected_vram, usd_path.
- When indexing heavy-asset docs, include small excerpt text and a link to asset manifests.

---

## Module 4 — Vision-Language-Action (VLA)
spec_id: SPEC-M4-VLA

Summary:
Module 4 covers the integration of vision, language, and action: speech-to-text with Whisper, LLM-based cognitive planning, converting natural language commands to ROS 2 action sequences, and multi-modal perception + instruction pipelines.

Learning Objectives:
- Convert voice input to text using Whisper or similar.
- Use LLMs to translate high-level natural language commands into a sequence of low-level robot actions (planning).
- Implement a control loop that accepts LLM-plans, validates them against safety constraints, and issues ROS 2 actions.
- Build tests to ensure the LLM planner does not produce unsafe or out-of-scope commands.

Scope:
- Speech recognition pipeline (Whisper integration).
- Planner patterns: LLM -> plan -> validator -> executor.
- Multi-modal perception (vision model + LLM) for object grounding.
- Safety & constraint checks.

Deliverables:
- Docs: docs/module-04-vla/ (whisper-integration, planning-patterns, safety, multimodal)
- Examples: /examples/module4/whisper_pipeline/, /examples/module4/planner_stub/
- Test harness: plan-safety validator unit tests.

Acceptance Criteria & Tests:
- Planner test: given a command like "pick up the red cup", planner returns a sequence of actions with object grounding metadata; validator confirms actions are within allowed set.
- Selection enforcement: in "selected_text" answer mode, the system prompt instructs LLM to only use selected text to generate plan reasoning.
- Retrieval & grounding: Q/A queries related to VLA must retrieve correct multimodal docs.

Sample Exercises:
- Lab 4.1: Implement a Whisper-based recorder that transcribes a 10-second clip and posts it to /api/answer as a question.
- Lab 4.2: Implement a simple planner that maps "go to X" to [TURN, WALK, STOP] steps and a validator that rejects "jump" commands.

Suggested Chapter Filenames:
- docs/module-04-vla/01-intro-vla.md
- docs/module-04-vla/02-whisper-and-speech.md
- docs/module-04-vla/03-llm-planning-patterns.md
- docs/module-04-vla/04-safety-and-validators.md
- docs/module-04-vla/exercises.md

Spec->Tasks mapping:
- TASK-M4-010: Add Module 4 docs and whisper/planner examples (SPEC-M4-VLA)
- TASK-M4-020: Index Module 4 docs (SPEC-M4-VLA)
- TASK-M4-030: Implement planner validator unit tests (SPEC-M4-VLA)

Indexing & Embedding Guidance:
- Provide example utterances and sample plans as separate chunks to ensure retrieval returns both natural language and structured plan artifacts.
- Tag safety rules and allowed action sets in metadata for faster retrieval by the answer pipeline.

---

# Module-Level Acceptance Test Suite (summary)
- For each module (M1–M4), include:
  - A small set of canonical Q/A pairs (10 per module) to be added to tests/acceptance/module-XX.json for RAG evaluation.
  - Index these Q/A pairs as part of the indexing CI step to produce expected retrievals.
  - Add unit tests that mock retrieval + LLM response to verify selection-mode enforcement.

# Mapping Modules to Release Plan
- MVP (v0.1-alpha): Minimal docs for Module 1 & 2 + RAG selection-mode working end to end.
- Beta (v0.2-beta): Add Module 3 content and simple planner examples.
- v1.0: Full Module 4, personalization, Urdu translations, Claude Code subagents.

# Notes for Implementers
- When creating docs for each module, include:
  - Learning objectives at top of page
  - Short summary (200–400 words)
  - Example code block(s) and expected outputs
  - Lab exercises with rubric
  - 3–5 Q/A pairs for acceptance tests

# References & Links
- Add external references and further reading in each module's docs, and include them in metadata for retrieval.
