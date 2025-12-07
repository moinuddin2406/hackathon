---
id: 001
title: Initial Constitution Creation
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: moinuddin
command: /sp.constitution
labels: ["constitution", "initialization", "governance", "hackathon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - None (constitution does not require tests)
---

## Prompt

Project Name: Physical AI & Humanoid Robotics — Essentials
Project Type: AI-native interactive textbook + free-tier RAG chatbot
Deployment: GitHub Pages (must be live and publicly accessible)
Hackathon Deadline: 30 Nov 2025, 6 PM (Pakistan Standard Time)

Core Purpose:
Create the world's first fully AI/spec-driven, minimalist, high-quality, open-access textbook that teaches the Panaversity Physical AI & Humanoid Robotics course in the fastest and clearest way possible — complete with an embedded RAG chatbot that answers ONLY from the book's content.

Scope (Exactly 6 short chapters — no more, no less):
1. Introduction to Physical AI
2. Basics of Humanoid Robotics
3. ROS 2 Fundamentals
4. Digital Twin Simulation (Gazebo + Isaac Sim)
5. Vision-Language-Action Systems
6. Capstone: Simple AI-Robot Pipeline

Core Principles (Non-negotiable):
1. Simplicity — Every sentence must be understandable on first read by an intermediate engineering student.
2. Accuracy — All technical statements must be correct, proven, and conservative (no hype, no speculation).
3. Minimalism — Remove anything that does not directly help the reader learn faster.
4. Speed — Full site + embeddings must build in under 2 minutes on GitHub Actions free tier.
5. Free-Tier Only — Every single component (hosting, DB, vector store, embeddings, backend) must run at $0/month.
6. Source-of-Truth RAG — Chatbot may ONLY retrieve and answer from the final rendered book text. No external knowledge, no training bleed.

Technical Constraints:
- Embedding model: ≤ 384 dimensions, < 300 MB (e.g., BGE-small-en-v1.5 or all-MiniLM-L6-v2)
- Vector DB: Qdrant Cloud free tier or equivalent
- Backend: FastAPI on Fly.io / Render / Cloudflare Workers free tier
- Frontend: Docusaurus classic template (minimal customisation)
- Total repo size: < 350 MB (including embeddings)

Mandatory Features (100 base points):
- Clean, responsive Docusaurus UI with perfect sidebar and dark mode
- "Select text → Ask AI" functionality that works on desktop and mobile
- RAG chatbot in navbar that cites exact chapter/section
- GitHub Actions workflow that builds and deploys to GitHub Pages in < 2 minutes
- Zero broken links (Docusaurus build must pass with onBrokenLinks: 'throw')

Bonus Features (up to +200 points):
- Better Auth signup/signin with background quiz → content personalization
- Per-chapter "Personalise for me" button (adjusts difficulty)
- Per-chapter "Urdu میں دکھائیں" toggle
- Reusable Claude Code subagents/skills used during creation

Governance:
- This constitution is the single source of truth.
- Every spec, prompt, and file must reference which principle/constraint it satisfies.
- Any deviation requires a GitHub issue + explicit justification.
- Constitution can only be amended via pull request titled "constitution: …"

Success Criteria (Go / No-Go):
- [ ] Live GitHub Pages URL works with zero errors
- [ ] RAG chatbot answers 20 random course questions correctly and cites sources
- [ ] "Select text → Ask AI" works perfectly
- [ ] Build time < 2 minutes on GitHub Actions free tier
- [ ] Total running cost = $0.00/month
- [ ] Demo video < 90 seconds ready

Commitment:
We pledge to deliver the simplest, most accurate, and fastest-learning Physical AI textbook humanity has ever seen — built entirely with free tools, for everyone, forever.

Signed by the Constitution — 7 December 2025

## Response snapshot

Successfully created the project constitution v1.0.0 at `.specify/memory/constitution.md`.

**Constitution Structure:**
- **6 Core Principles**: Simplicity, Accuracy, Minimalism, Speed, Free-Tier Only, Source-of-Truth RAG
- **Project Scope**: 6 chapters covering Physical AI & Humanoid Robotics fundamentals
- **Technical Constraints**: Free-tier components, < 2 min build time, < 350 MB repo size
- **Mandatory Features**: Docusaurus UI, RAG chatbot, text selection AI, GitHub Actions CI/CD
- **Bonus Features**: Better Auth personalization, per-chapter customization, Urdu translation
- **Governance**: Constitution as single source of truth, amendment process via PR

**Key Decisions:**
1. **Version 1.0.0**: Initial ratification
2. **Ratification Date**: 2025-12-07
3. **All placeholders filled**: No unresolved template tokens
4. **Sync Impact Report**: Added as HTML comment at top of constitution

**Template Alignment Verified:**
- ✅ spec-template.md
- ✅ plan-template.md
- ✅ tasks-template.md
- ✅ phr-template.prompt.md
- ✅ adr-template.md

## Outcome

- ✅ Impact: Project constitution established as single source of truth; all development work must now reference constitutional principles
- 🧪 Tests: None (constitution is governance document)
- 📁 Files: Created .specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: /sp.specify to create first feature specification; /sp.plan for architectural planning
- 🧠 Reflection: Constitution successfully captures all user requirements with clear, testable principles. The 6 core principles provide strong guardrails for hackathon development: Simplicity ensures learning effectiveness, Accuracy builds trust, Minimalism prevents scope creep, Speed enables rapid iteration, Free-Tier ensures sustainability, and Source-of-Truth RAG maintains answer quality. Governance section establishes clear amendment process and compliance requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation PASS (all placeholders filled, version semantics correct, governance complete)
- Prompt variant (if applicable): Standard constitution creation
- Next experiment (smallest change to try): Monitor constitution compliance during feature development; validate that principles prevent common hackathon pitfalls (scope creep, over-engineering, vendor lock-in)
