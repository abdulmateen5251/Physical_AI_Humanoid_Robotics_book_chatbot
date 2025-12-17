<!--
Sync Impact Report - Constitution v2.0.0 (2025-12-16)
=======================================================
Version Change: v1.0.0 → v2.0.0 (MAJOR - Complete project pivot)

Modified Principles:
- All principles rewritten for new PDCP project (complete pivot from Physical AI/Robotics to Developer Content Platform)

Added Sections:
- Principle 1: Spec-Driven Development
- Principle 2: Privacy & Consent by Design  
- Principle 3: Security First
- Principle 4: Accessibility & Inclusivity
- Principle 5: Observability & Learning
- Principle 6: Automation & Quality
- Decision-Making framework
- Roles definitions (Maintainers, Contributors, Security/Privacy Leads)
- Change Management process
- Non-Negotiables section

Removed Sections:
- Vision Principles (replaced with Core Principles)
- Scope/Non-Goals/Constraints (moved to spec.md as appropriate)
- Success Criteria (moved to spec.md)

Templates Requiring Updates:
✅ plan-template.md - Constitution Check section aligns
✅ spec-template.md - User story requirements align  
✅ tasks-template.md - Task categorization aligns
⚠ Command files - May need review for PDCP-specific guidance

Follow-up TODOs:
- Ratification date set to 2025-12-16 (initial adoption)
- Review all active specs for alignment with new principles
-->

# Project Constitution — Personalized Developer Content Platform

**Version**: 2.0.0  
**Ratification Date**: 2025-12-16  
**Last Amended**: 2025-12-16

## Project Name

Personalized Developer Content Platform (PDCP)

## Mission

Help developers learn and build more effectively by tailoring content to their software and hardware context. Respect user privacy and provide transparent control over profiling and personalization.

## Core Principles

### Principle 1: Spec-Driven Development

**Rule**: spec.md is the source of truth for all feature work. No feature development proceeds without an approved specification.

**Rationale**: Spec-driven development ensures alignment, traceability, and prevents scope drift. Every feature must have clear acceptance criteria before implementation begins.

**Requirements**:
- `spec.md` MUST be created and approved before `plan.md` and `tasks.md` are generated.
- `plan.md` and `tasks.md` MUST remain synchronized with `spec.md` at all times.
- Any material change to feature requirements MUST be reflected in `spec.md` first.
- PRs MUST reference the associated spec and task IDs.

### Principle 2: Privacy & Consent by Design

**Rule**: Collect only data necessary for personalization. Users MUST explicitly opt-in before any tracking or profiling occurs.

**Rationale**: Privacy is a fundamental right. Developers trust us with sensitive context about their environment; we must honor that trust with transparency and control.

**Requirements**:
- Data collection MUST be minimal and purpose-specific.
- Users MUST be clearly informed about what data is collected and how it is used.
- Opt-in consent MUST be required for profiling, analytics, and personalization features.
- Opt-out MUST be honored immediately and completely.
- Sensitive data (credentials, proprietary code) MUST be stored securely with minimal retention.
- Data deletion requests MUST be fulfilled within 30 days.

### Principle 3: Security First

**Rule**: Integrate authentication via Better Auth. Follow least privilege and secure defaults for all features.

**Rationale**: Security vulnerabilities undermine user trust and platform integrity. Proactive security measures prevent costly breaches.

**Requirements**:
- Better Auth MUST be used for all authentication flows.
- Secrets (API keys, tokens) MUST NEVER be committed to source control.
- Secrets MUST be stored in environment variables or secure vaults (GitHub Secrets, etc.).
- Least privilege MUST be applied to all data access and API endpoints.
- Threat modeling MUST be performed for new capabilities involving user data or external integrations.
- Security reviews MUST be conducted by Security/Privacy Leads before merging changes to auth or data handling.

### Principle 4: Accessibility & Inclusivity

**Rule**: User interfaces MUST meet WCAG AA standards. Provide alternative paths when hardware acceleration or advanced features are unavailable.

**Rationale**: Accessibility broadens reach and ensures equitable access. Not all developers have cutting-edge hardware or perfect vision.

**Requirements**:
- UI components MUST comply with WCAG 2.1 Level AA.
- Content MUST be navigable via keyboard.
- Alternative text MUST be provided for images and visual elements.
- Graceful degradation MUST be implemented when hardware acceleration is absent.
- Testing MUST include accessibility audits using automated tools (e.g., axe, Lighthouse).

### Principle 5: Observability & Learning

**Rule**: Define success metrics before feature development. Instrument events with privacy-respecting analytics.

**Rationale**: Data-driven decisions improve product quality. Observability helps us understand what works and what doesn't, but must not compromise privacy.

**Requirements**:
- Success metrics MUST be defined in `spec.md` before implementation.
- Analytics events MUST respect user consent and privacy settings.
- Personally Identifiable Information (PII) MUST NOT be logged or tracked without explicit consent.
- Metrics dashboards MUST be accessible to maintainers for continuous improvement.
- Post-launch reviews MUST compare actual metrics against success criteria.

### Principle 6: Automation & Quality

**Rule**: CI enforces linting, tests, and documentation synchronization. PRs MUST include tests and update docs when behavior changes.

**Rationale**: Automation prevents regressions and maintains code quality. Manual processes are error-prone and do not scale.

**Requirements**:
- CI pipelines MUST run linting, unit tests, and integration tests on every PR.
- PRs that change behavior MUST include corresponding test updates.
- PRs that change user-facing features MUST update documentation.
- Documentation synchronization checks MUST be automated in CI.
- Breaking changes MUST be flagged and approved by maintainers before merge.

## Decision-Making

**Product Scope Decisions**:
- All product scope decisions MUST originate in `spec.md` PRs.
- Disagreements MUST be resolved by referencing Core Principles and user impact analysis.
- If consensus cannot be reached, escalate to the Maintainer group for a timeboxed decision (72 hours maximum).

**Technical Decisions**:
- Technical decisions should prioritize simplicity, security, and maintainability.
- Architectural Decision Records (ADRs) MUST be created for significant technical choices.

## Roles

**Maintainers**:
- Approve specification changes and releases.
- Enforce compliance with Core Principles.
- Resolve escalated decision-making disputes.
- Conduct final reviews before production deployment.

**Contributors**:
- Propose changes through PRs aligned with the spec-driven workflow.
- Ensure PRs include tests, documentation updates, and reference spec/task IDs.
- Participate in code reviews and provide constructive feedback.

**Security/Privacy Leads**:
- Review changes impacting authentication, data collection, and tracking.
- Conduct threat modeling for new capabilities.
- Validate compliance with Privacy & Consent by Design and Security First principles.
- Approve security-sensitive PRs before merge.

## Change Management

**Specification Changes**:
- Any material change to feature requirements MUST be submitted as a `spec.md` PR.
- `plan.md` and `tasks.md` MUST be updated in the same PR or in a follow-up PR explicitly referenced as dependent.

**Breaking Changes**:
- Breaking changes MUST be documented in the PR description and changelog.
- Deprecation warnings MUST be issued at least one release cycle before removal.
- Migration guides MUST be provided for users affected by breaking changes.

## Non-Negotiables

The following constraints MUST NEVER be violated:

1. **No tracking without explicit consent**: Users MUST opt-in before any profiling or analytics occur.
2. **No silent changes to authentication or data collection**: Changes to auth flows or data handling MUST be announced and reviewed by Security/Privacy Leads.
3. **No production secrets in code or logs**: Secrets MUST be stored securely and MUST NEVER appear in source code, logs, or error messages.

## Amendments

**Process**:
- Amendments to this Constitution MUST be proposed via PR.
- A 72-hour review window MUST be observed for all amendment PRs.
- Amendments MUST receive explicit approval from at least two Maintainers before merge.
- Amendment PRs MUST include a Sync Impact Report detailing affected templates and follow-up actions.

**Versioning**:
- Constitution version follows semantic versioning (MAJOR.MINOR.PATCH).
- MAJOR: Backward-incompatible governance changes or principle removals/redefinitions.
- MINOR: New principles or materially expanded guidance.
- PATCH: Clarifications, wording improvements, non-semantic refinements.

## Licensing & Attribution

- **Code**: MIT License (default).
- **Content**: Creative Commons Attribution-ShareAlike 4.0 (CC-BY-SA 4.0).
- Attribution sections MUST credit SpecKit+, Better Auth, and any third-party dependencies.